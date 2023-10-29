#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


using namespace std;
using namespace rclcpp;

class BallRollingTracker : public Node
{
public:
  BallRollingTracker()
    : Node("ball_rolling_tracker")
  {
    // Subscribe to the imu_msg topic.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_msg", 10, std::bind(&BallRollingTracker::imuCallback, this, std::placeholders::_1));

    // Subscribe to the odom topic.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&BallRollingTracker::odomCallback, this, std::placeholders::_1));

    // Create a TF2 broadcaster and listener.
    tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(this);

    // Create a variable to store the current position of the ball.
    ball_position_ = tf2::Vector3(0, 0, 0);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Convert the IMU message to a tf2::Twist object.
    geometry_msgs::msg::Twist twist;
    tf2_sensor_msgs::TwistFromImuMsg(*msg, twist);


    // Broadcast the tf2::Twist object to TF2.
    tf2_broadcaster_->sendTransform(
      tf2::TransformStamped(twist, msg->header.stamp, "base_link", "ball_frame"));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Get the current position of the ball using the transform_lookup service.
    tf2::TransformStamped transform_stamped;
    tf2_listener_->lookupTransform("base_link", "ball_frame", msg->header.stamp, transform_stamped);
    ball_position_ = transform_stamped.transform.translation;

    // Use the tf2::Twist object from the imuCallback to estimate the new position of the ball.
    tf2::Vector3 new_ball_position = ball_position_ + twist.linear.x * msg->header.stamp.sec;

    // Publish the new position of the ball to the odom topic.
    nav_msgs::msg::Odometry new_odom_msg;
    new_odom_msg.header.stamp = msg->header.stamp;
    new_odom_msg.pose.pose.position = new_ball_position;

    publish("odom", new_odom_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  tf2::Vector3 ball_position_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BallRollingTracker>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
