#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class ImuToOdom : public rclcpp::Node
{
public:
    ImuToOdom()
        : Node("imu_to_odom"), x_(0.0), y_(0.0), yaw_(0.0), vx_(0.0), vy_(0.0), vth_(0.0)
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ImuToOdom::imuCallback, this, std::placeholders::_1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        previous_time_ = this->now();
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - previous_time_).seconds();

        vth_ += msg->angular_velocity.z * dt;
        vx_ += msg->linear_acceleration.x * dt;
        vy_ += msg->linear_acceleration.y * dt;

        x_ += vx_ * dt * cos(vth_);
        y_ += vx_ * dt * sin(vth_);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, vth_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = vx_;
        odom_msg.twist.twist.linear.y = vy_;
        odom_msg.twist.twist.angular.z = vth_;

        odom_publisher_->publish(odom_msg);

        // Broadcast the transform
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = current_time;
        odom_transform.header.frame_id = "odom";
        odom_transform.child_frame_id = "base_link";

        odom_transform.transform.translation.x = x_;
        odom_transform.transform.translation.y = y_;
        odom_transform.transform.translation.z = 0.0;
        odom_transform.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_transform);

        previous_time_ = current_time;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, yaw_, vx_, vy_, vth_;
    rclcpp::Time previous_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToOdom>());
    rclcpp::shutdown();
    return 0;
}
