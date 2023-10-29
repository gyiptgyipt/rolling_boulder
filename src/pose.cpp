#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"

class PosePublisher : public rclcpp::Node {
public:
  PosePublisher() : Node("pose_publisher") {
    pose_publisher = create_publisher<geometry_msgs::msg::Pose>("pose", 10);
  }
            imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&PosePublisher::imuCallback, this, std::placeholders::_1));



private:

    void publish(const std::shared_ptr<sensor_msgs::msg::Imu> msg) {

    auto acc_x = msg->linear_acceleration.x;
    auto acc_y = msg->linear_acceleration.y;
    auto acc_z = msg->linear_acceleration.z;


    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    pose_publisher->publish(pose);
  }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
  
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PosePublisher>();

  // Publish a Pose message every second
  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(1.0s, [node]() { node->publish(); });

  // Spin until the node is shutdown
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
