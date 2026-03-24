#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotionTestNode : public rclcpp::Node {
 public:
  MotionTestNode() : Node("motion_test_node") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ =
        this->create_wall_timer(50ms, std::bind(&MotionTestNode::loop, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Motion test started");
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;

  enum State { MOVE_FORWARD, ROTATE, DONE } state_ = MOVE_FORWARD;

  void loop() {
    auto now = this->now();
    double t = (now - start_time_).seconds();

    geometry_msgs::msg::Twist cmd;

    // ===== 参数 =====
    double v = 0.2;  // m/s
    double w = 0.5;  // rad/s

    double move_time = 1.0 / v;       // 走1m
    double rotate_time = M_PI_2 / w;  // 90°

    switch (state_) {
      case MOVE_FORWARD:
        if (t < move_time) {
          cmd.linear.x = v;
        } else {
          state_ = ROTATE;
          start_time_ = now;
          RCLCPP_INFO(this->get_logger(), "Start rotate");
        }
        break;

      case ROTATE:
        if (t < rotate_time) {
          cmd.angular.z = w;
        } else {
          state_ = DONE;
          RCLCPP_INFO(this->get_logger(), "Done");
        }
        break;

      case DONE:
        // 停车
        break;
    }

    pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionTestNode>());
  rclcpp::shutdown();
  return 0;
}