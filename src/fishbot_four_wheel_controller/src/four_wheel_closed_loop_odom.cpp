#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FourWheelClosedLoopOdom : public rclcpp::Node {
 public:
  FourWheelClosedLoopOdom() : Node("four_wheel_closed_loop_odom") {
    wheel_radius_ = this->declare_parameter("wheel_radius", 0.06);

    odom_frame_id_ = this->declare_parameter("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter("base_frame_id", "base_footprint");
    joint_states_topic_ =
        this->declare_parameter("joint_states_topic", "/joint_states");
    publish_tf_ = this->declare_parameter("publish_tf", true);
    joint_state_timeout_sec_ =
        this->declare_parameter("joint_state_timeout_sec", 0.2);

    // 轮位坐标，直接按你 xacro 中四个 steer_joint 的 origin 提取
    // 顺序：前左、前右、后左、后右
    wheel_x_[0] = this->declare_parameter("front_left_x", 0.225);
    wheel_y_[0] = this->declare_parameter("front_left_y", 0.21);

    wheel_x_[1] = this->declare_parameter("front_right_x", 0.225);
    wheel_y_[1] = this->declare_parameter("front_right_y", -0.21);

    wheel_x_[2] = this->declare_parameter("rear_left_x", -0.225);
    wheel_y_[2] = this->declare_parameter("rear_left_y", 0.21);

    wheel_x_[3] = this->declare_parameter("rear_right_x", -0.225);
    wheel_y_[3] = this->declare_parameter("rear_right_y", -0.21);

    // 如果你后面发现仿真里某些轮速度方向和预期相反，
    // 可以直接改这些符号参数，而不用改代码
    wheel_dir_sign_[0] =
        this->declare_parameter("front_left_wheel_dir_sign", 1.0);
    wheel_dir_sign_[1] =
        this->declare_parameter("front_right_wheel_dir_sign", 1.0);
    wheel_dir_sign_[2] =
        this->declare_parameter("rear_left_wheel_dir_sign", 1.0);
    wheel_dir_sign_[3] =
        this->declare_parameter("rear_right_wheel_dir_sign", 1.0);

    steer_joint_names_[0] = "front_left_steer_joint";
    steer_joint_names_[1] = "front_right_steer_joint";
    steer_joint_names_[2] = "rear_left_steer_joint";
    steer_joint_names_[3] = "rear_right_steer_joint";

    wheel_joint_names_[0] = "front_left_wheel_joint";
    wheel_joint_names_[1] = "front_right_wheel_joint";
    wheel_joint_names_[2] = "rear_left_wheel_joint";
    wheel_joint_names_[3] = "rear_right_wheel_joint";

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 50,
        std::bind(&FourWheelClosedLoopOdom::joint_state_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&FourWheelClosedLoopOdom::timer_callback, this));

    last_update_time_ = this->now();
    last_joint_state_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "four_wheel_closed_loop_odom started");
  }

 private:
  struct WheelState {
    double steer_angle{0.0};  // 转向角
    double wheel_omega{0.0};  // 轮角速度 rad/s
    bool steer_valid{false};
    bool wheel_valid{false};
  };

  static double normalize_angle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  }

  // 求解 3x3 线性方程组 A x = b
  // 使用高斯消元，成功返回 true
  static bool solve_3x3(double A[3][3], double b[3], double x[3]) {
    constexpr double kEps = 1e-10;

    double M[3][4] = {
        {A[0][0], A[0][1], A[0][2], b[0]},
        {A[1][0], A[1][1], A[1][2], b[1]},
        {A[2][0], A[2][1], A[2][2], b[2]},
    };

    for (int col = 0; col < 3; ++col) {
      int pivot = col;
      for (int row = col + 1; row < 3; ++row) {
        if (std::abs(M[row][col]) > std::abs(M[pivot][col])) {
          pivot = row;
        }
      }

      if (std::abs(M[pivot][col]) < kEps) {
        return false;
      }

      if (pivot != col) {
        for (int k = col; k < 4; ++k) {
          std::swap(M[col][k], M[pivot][k]);
        }
      }

      const double div = M[col][col];
      for (int k = col; k < 4; ++k) {
        M[col][k] /= div;
      }

      for (int row = 0; row < 3; ++row) {
        if (row == col) {
          continue;
        }
        const double factor = M[row][col];
        for (int k = col; k < 4; ++k) {
          M[row][k] -= factor * M[col][k];
        }
      }
    }

    x[0] = M[0][3];
    x[1] = M[1][3];
    x[2] = M[2][3];
    return true;
  }

  // 根据四个轮子的实际状态，闭环反解底盘速度 vx, vy, wz
  // 每个轮子有两条约束：
  // vix = vx - wz * y_i
  // viy = vy + wz * x_i
  //
  // 其中：
  // vix = v_i * cos(theta_i)
  // viy = v_i * sin(theta_i)
  bool compute_body_twist_from_wheels(double &vx, double &vy, double &wz) {
    double AtA[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double Atb[3] = {0.0, 0.0, 0.0};

    int equation_count = 0;

    for (size_t i = 0; i < 4; ++i) {
      if (!wheel_states_[i].steer_valid || !wheel_states_[i].wheel_valid) {
        continue;
      }

      const double theta = wheel_states_[i].steer_angle;
      const double omega = wheel_states_[i].wheel_omega * wheel_dir_sign_[i];
      const double v = omega * wheel_radius_;

      const double vix = v * std::cos(theta);
      const double viy = v * std::sin(theta);

      const double x_i = wheel_x_[i];
      const double y_i = wheel_y_[i];

      // 方程 1: [1, 0, -y_i] [vx vy wz]^T = vix
      {
        const double a[3] = {1.0, 0.0, -y_i};
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            AtA[r][c] += a[r] * a[c];
          }
          Atb[r] += a[r] * vix;
        }
        ++equation_count;
      }

      // 方程 2: [0, 1, x_i] [vx vy wz]^T = viy
      {
        const double a[3] = {0.0, 1.0, x_i};
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            AtA[r][c] += a[r] * a[c];
          }
          Atb[r] += a[r] * viy;
        }
        ++equation_count;
      }
    }

    if (equation_count < 3) {
      return false;
    }

    double result[3] = {0.0, 0.0, 0.0};
    if (!solve_3x3(AtA, Atb, result)) {
      return false;
    }

    vx = result[0];
    vy = result[1];
    wz = result[2];
    return true;
  }

  void publish_odometry(const rclcpp::Time &stamp, double vx, double vy,
                        double wz) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_yaw_);
    q.normalize();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = wz;

    // 基础协方差，可后续按仿真表现再调
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[14] = 1e6;   // z
    odom_msg.pose.covariance[21] = 1e6;   // roll
    odom_msg.pose.covariance[28] = 1e6;   // pitch
    odom_msg.pose.covariance[35] = 0.02;  // yaw

    odom_msg.twist.covariance[0] = 0.02;
    odom_msg.twist.covariance[7] = 0.02;
    odom_msg.twist.covariance[14] = 1e6;
    odom_msg.twist.covariance[21] = 1e6;
    odom_msg.twist.covariance[28] = 1e6;
    odom_msg.twist.covariance[35] = 0.04;

    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_id_;
      tf_msg.child_frame_id = base_frame_id_;
      tf_msg.transform.translation.x = odom_x_;
      tf_msg.transform.translation.y = odom_y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const size_t n = msg->name.size();
    if (msg->position.size() < n || msg->velocity.size() < n) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "joint_states position/velocity size mismatch");
      return;
    }

    std::unordered_map<std::string, size_t> name_to_index;
    name_to_index.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      name_to_index[msg->name[i]] = i;
    }

    bool all_found = true;

    for (size_t i = 0; i < 4; ++i) {
      const auto steer_it = name_to_index.find(steer_joint_names_[i]);
      if (steer_it != name_to_index.end()) {
        const size_t idx = steer_it->second;
        wheel_states_[i].steer_angle = msg->position[idx];
        wheel_states_[i].steer_valid = true;
      } else {
        wheel_states_[i].steer_valid = false;
        all_found = false;
      }

      const auto wheel_it = name_to_index.find(wheel_joint_names_[i]);
      if (wheel_it != name_to_index.end()) {
        const size_t idx = wheel_it->second;
        wheel_states_[i].wheel_omega = msg->velocity[idx];
        wheel_states_[i].wheel_valid = true;
      } else {
        wheel_states_[i].wheel_valid = false;
        all_found = false;
      }
    }

    if (!all_found) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Some required joints are missing in /joint_states");
      return;
    }

    has_joint_state_ = true;

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      last_joint_state_time_ = this->now();
    } else {
      last_joint_state_time_ = rclcpp::Time(msg->header.stamp);
    }
  }

  void timer_callback() {
    const rclcpp::Time now = this->now();
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    if (dt <= 0.0) {
      return;
    }

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    bool use_zero_twist = true;

    if (has_joint_state_) {
      const double age = (now - last_joint_state_time_).seconds();
      if (age <= joint_state_timeout_sec_) {
        if (compute_body_twist_from_wheels(vx, vy, wz)) {
          use_zero_twist = false;
        }
      }
    }

    if (use_zero_twist) {
      vx = 0.0;
      vy = 0.0;
      wz = 0.0;
    }

    // 机器人坐标系速度 -> odom 坐标系积分
    const double cos_yaw = std::cos(odom_yaw_);
    const double sin_yaw = std::sin(odom_yaw_);

    odom_x_ += (vx * cos_yaw - vy * sin_yaw) * dt;
    odom_y_ += (vx * sin_yaw + vy * cos_yaw) * dt;
    odom_yaw_ = normalize_angle(odom_yaw_ + wz * dt);

    publish_odometry(now, vx, vy, wz);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};
  std::string joint_states_topic_{"/joint_states"};

  bool publish_tf_{true};
  bool has_joint_state_{false};

  double wheel_radius_{0.06};
  double joint_state_timeout_sec_{0.2};

  std::array<double, 4> wheel_x_{};
  std::array<double, 4> wheel_y_{};
  std::array<double, 4> wheel_dir_sign_{};

  std::array<std::string, 4> steer_joint_names_{};
  std::array<std::string, 4> wheel_joint_names_{};
  std::array<WheelState, 4> wheel_states_{};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

  rclcpp::Time last_update_time_;
  rclcpp::Time last_joint_state_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FourWheelClosedLoopOdom>());
  rclcpp::shutdown();
  return 0;
}