#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FourWheelCommander : public rclcpp::Node {
 public:
  FourWheelCommander() : Node("four_wheel_commander") {
    wheel_separation_ = this->declare_parameter("wheel_separation", 0.42);
    wheel_base_ = this->declare_parameter("wheel_base", 0.45);
    wheel_radius_ = this->declare_parameter("wheel_radius", 0.06);
    wheel_steering_y_offset_ =
        this->declare_parameter("wheel_steering_y_offset", 0.0);
    joy_linear_x_gain_ = this->declare_parameter("joy_linear_x_gain", 1.0);
    joy_linear_y_gain_ = this->declare_parameter("joy_linear_y_gain", 1.0);
    joy_angular_z_gain_ = this->declare_parameter("joy_angular_z_gain", 1.0);
    cmd_vel_timeout_sec_ = this->declare_parameter("cmd_vel_timeout_sec", 0.0);
    joy_timeout_sec_ = this->declare_parameter("joy_timeout_sec", 0.2);
    linear_deadband_ = this->declare_parameter("linear_deadband", 0.02);
    angular_deadband_ = this->declare_parameter("angular_deadband", 0.02);

    steering_track_ = wheel_separation_ - 2.0 * wheel_steering_y_offset_;

    pub_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10);
    pub_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_velocity_controller/commands", 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&FourWheelCommander::cmd_vel_callback, this,
                  std::placeholders::_1));

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/teleop_cmd_vel", 10,
        std::bind(&FourWheelCommander::joy_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&FourWheelCommander::timer_callback, this));

    last_cmd_vel_time_ = this->now();
    last_joy_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "four_wheel_commander started");
  }

 private:
  struct WheelCommand {
    double steering{0.0};
    double speed{0.0};
  };

  enum class ActiveSource {
    kNone,
    kCmdVel,
    kJoy,
  };

  static double apply_deadband(double value, double deadband) {
    return (std::abs(value) < deadband) ? 0.0 : value;
  }

  static WheelCommand normalize_wheel_command(double steering, double speed) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kHalfPi = 1.57079632679489661923;

    while (steering > kPi) {
      steering -= 2.0 * kPi;
    }
    while (steering < -kPi) {
      steering += 2.0 * kPi;
    }

    // Keep steering inside [-pi/2, pi/2] to respect steering joint limits.
    if (steering > kHalfPi) {
      steering -= kPi;
      speed = -speed;
    } else if (steering < -kHalfPi) {
      steering += kPi;
      speed = -speed;
    }

    return {steering, speed};
  }

  void compute_swerve_from_cmd_vel() {
    constexpr double kEps = 1e-6;
    const double vx = vel_msg_.linear.x;
    const double vy = vel_msg_.linear.y;
    const double wz = vel_msg_.angular.z;

    const double half_wheel_base = wheel_base_ * 0.5;
    const double half_track = steering_track_ * 0.5;

    // Wheel center positions: FL, FR, RL, RR.
    const std::array<std::pair<double, double>, 4> wheel_xy = {
        std::make_pair(half_wheel_base, half_track),
        std::make_pair(half_wheel_base, -half_track),
        std::make_pair(-half_wheel_base, half_track),
        std::make_pair(-half_wheel_base, -half_track),
    };

    for (size_t i = 0; i < wheel_xy.size(); ++i) {
      const double x = wheel_xy[i].first;
      const double y = wheel_xy[i].second;

      const double wheel_vx = vx - wz * y;
      const double wheel_vy = vy + wz * x;
      const double wheel_speed_mps = std::hypot(wheel_vx, wheel_vy);

      if (wheel_speed_mps < kEps) {
        pos_[i] = 0.0;
        vel_[i] = 0.0;
        continue;
      }

      const double raw_steer = std::atan2(wheel_vy, wheel_vx);
      const double raw_speed = wheel_speed_mps / wheel_radius_;
      const auto cmd = normalize_wheel_command(raw_steer, raw_speed);
      pos_[i] = cmd.steering;
      vel_[i] = cmd.speed;
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    joy_mode_selection_ = 4;
    joy_vel_msg_ = geometry_msgs::msg::Twist();

    // Xbox mapping: A->in-phase, LB->opposite-phase, RB->pivot
    if (msg->buttons.size() > 5U) {
      if (msg->buttons[0] == 1) {
        joy_mode_selection_ = 2;
      } else if (msg->buttons[4] == 1) {
        joy_mode_selection_ = 1;
      } else if (msg->buttons[5] == 1) {
        joy_mode_selection_ = 3;
      }
    }

    if (msg->axes.size() > 3U) {
      joy_vel_msg_.linear.x =
          apply_deadband(msg->axes[1] * joy_linear_x_gain_, linear_deadband_);
      joy_vel_msg_.linear.y =
          apply_deadband(msg->axes[0] * joy_linear_y_gain_, linear_deadband_);
      joy_vel_msg_.angular.z =
          apply_deadband(msg->axes[3] * joy_angular_z_gain_, angular_deadband_);
    }

    has_joy_command_ = true;
    last_joy_time_ = this->now();
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // cmd_vel uses full four-wheel kinematics (vx, vy, wz) in auto mode.
    cmd_vel_msg_ = *msg;
    cmd_vel_msg_.linear.x =
        apply_deadband(cmd_vel_msg_.linear.x, linear_deadband_);
    cmd_vel_msg_.linear.y =
        apply_deadband(cmd_vel_msg_.linear.y, linear_deadband_);
    cmd_vel_msg_.angular.z =
        apply_deadband(cmd_vel_msg_.angular.z, angular_deadband_);
    has_cmd_vel_command_ = true;
    last_cmd_vel_time_ = this->now();
  }

  void timer_callback() {
    constexpr double kEps = 1e-6;
    pos_.fill(0.0);
    vel_.fill(0.0);

    if (cmd_vel_timeout_sec_ > 0.0) {
      if (has_cmd_vel_command_ &&
          (this->now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_sec_) {
        has_cmd_vel_command_ = false;
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
      }
    }

    if (joy_timeout_sec_ > 0.0) {
      if (has_joy_command_ &&
          (this->now() - last_joy_time_).seconds() > joy_timeout_sec_) {
        has_joy_command_ = false;
        joy_vel_msg_ = geometry_msgs::msg::Twist();
        joy_mode_selection_ = 4;
      }
    }

    active_source_ = ActiveSource::kNone;
    if (has_cmd_vel_command_) {
      // /cmd_vel has higher priority when both sources are present.
      active_source_ = ActiveSource::kCmdVel;
      vel_msg_ = cmd_vel_msg_;
      mode_selection_ = 0;
    } else if (has_joy_command_) {
      active_source_ = ActiveSource::kJoy;
      vel_msg_ = joy_vel_msg_;
      mode_selection_ = joy_mode_selection_;
    }

    if (active_source_ == ActiveSource::kNone) {
      std_msgs::msg::Float64MultiArray pos_array;
      std_msgs::msg::Float64MultiArray vel_array;
      pos_array.data = std::vector<double>(pos_.begin(), pos_.end());
      vel_array.data = std::vector<double>(vel_.begin(), vel_.end());
      pub_pos_->publish(pos_array);
      pub_vel_->publish(vel_array);
      return;
    }

    if (mode_selection_ == 0) {
      compute_swerve_from_cmd_vel();
    }
    // opposite phase
    else if (mode_selection_ == 1) {
      const double vx = vel_msg_.linear.x;
      const double wz = vel_msg_.angular.z;
      const double sign = (std::abs(vx) < kEps) ? 1.0 : std::copysign(1.0, vx);
      const double steer_offset = wz * wheel_steering_y_offset_;

      const double left_speed =
          sign *
          std::hypot(vx - wz * steering_track_ / 2.0, wz * wheel_base_ / 2.0);
      const double right_speed =
          sign *
          std::hypot(vx + wz * steering_track_ / 2.0, wz * wheel_base_ / 2.0);

      pos_[0] = std::atan((wz * wheel_base_) /
                          (2.0 * vx + wz * steering_track_ + kEps));
      pos_[1] = std::atan((wz * wheel_base_) /
                          (2.0 * vx - wz * steering_track_ + kEps));
      pos_[2] = -pos_[0];
      pos_[3] = -pos_[1];

      vel_[0] = (left_speed - steer_offset) / wheel_radius_;
      vel_[1] = (right_speed + steer_offset) / wheel_radius_;
      vel_[2] = (left_speed - steer_offset) / wheel_radius_;
      vel_[3] = (right_speed + steer_offset) / wheel_radius_;
    }
    // in-phase
    else if (mode_selection_ == 2) {
      const double vx = vel_msg_.linear.x;
      const double vy = vel_msg_.linear.y;
      const double v = std::hypot(vx, vy);

      if (v > kEps) {
        const double heading = std::atan2(vy, vx);
        pos_[0] = heading;
        pos_[1] = heading;
        pos_[2] = heading;
        pos_[3] = heading;

        const double wheel_speed = v / wheel_radius_;
        vel_[0] = wheel_speed;
        vel_[1] = wheel_speed;
        vel_[2] = wheel_speed;
        vel_[3] = wheel_speed;
      }
    }
    // pivot turn
    else if (mode_selection_ == 3) {
      const double wz = vel_msg_.angular.z;
      const double steer_ang = std::atan2(wheel_base_, steering_track_ + kEps);
      const double tangential_v =
          std::abs(wz) * std::hypot(wheel_base_ / 2.0, steering_track_ / 2.0);
      const double wheel_speed = tangential_v / wheel_radius_;

      pos_[0] = -steer_ang;
      pos_[1] = steer_ang;
      pos_[2] = steer_ang;
      pos_[3] = -steer_ang;

      const double sign = (wz >= 0.0) ? 1.0 : -1.0;
      vel_[0] = -sign * wheel_speed;
      vel_[1] = sign * wheel_speed;
      vel_[2] = -sign * wheel_speed;
      vel_[3] = sign * wheel_speed;
    }

    std_msgs::msg::Float64MultiArray pos_array;
    std_msgs::msg::Float64MultiArray vel_array;
    pos_array.data = std::vector<double>(pos_.begin(), pos_.end());
    vel_array.data = std::vector<double>(vel_.begin(), vel_.end());
    pub_pos_->publish(pos_array);
    pub_vel_->publish(vel_array);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist cmd_vel_msg_;
  geometry_msgs::msg::Twist joy_vel_msg_;
  geometry_msgs::msg::Twist vel_msg_;
  int mode_selection_{4};
  int joy_mode_selection_{4};

  double wheel_separation_{0.42};
  double wheel_base_{0.45};
  double wheel_radius_{0.06};
  double wheel_steering_y_offset_{0.0};
  double steering_track_{0.42};
  double joy_linear_x_gain_{1.0};
  double joy_linear_y_gain_{1.0};
  double joy_angular_z_gain_{1.0};
  double cmd_vel_timeout_sec_{0.0};
  double joy_timeout_sec_{0.2};
  double linear_deadband_{0.02};
  double angular_deadband_{0.02};
  bool has_cmd_vel_command_{false};
  bool has_joy_command_{false};
  ActiveSource active_source_{ActiveSource::kNone};
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_joy_time_;

  std::array<double, 4> pos_{};
  std::array<double, 4> vel_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FourWheelCommander>());
  rclcpp::shutdown();
  return 0;
}
