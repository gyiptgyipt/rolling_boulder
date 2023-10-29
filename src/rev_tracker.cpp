#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"


class rev_tracker : public rclcpp::Node {
public:
    rev_tracker(): Node("rev_tracker") {
              
        // publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/imu_quaternion", 10);
        
        imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/imu/quaternion", 10, std::bind(&rev_tracker::imuCallback, this, std::placeholders::_1));



    }

private:

    int full_rotations = 0;
    double pi_vel = 3.0415;
    double rotation_count(double sensor_data) {
    // Check for wraparound (more than pi radians change)
    if (sensor_data   < -pi_vel) {
        full_rotations++;

    } else if (sensor_data> pi_vel) {
        full_rotations--;
    }
    }
    void imuCallback( const std::shared_ptr<geometry_msgs::msg::Quaternion> msg) {
       

        geometry_msgs::msg::TransformStamped transform_stamped;
        geometry_msgs::msg::Twist twist;
        
        transform_stamped.header.stamp = this->get_clock()->now();

        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time when = this->get_clock()->now() - rclcpp::Duration(5, 0);
        rclcpp::Time t_delta = now + rclcpp::Duration(1/10, 0);

        double time_now = now.seconds();

        auto wheel_radius = 0.0508;

        auto roll    = atan2(2 * (msg->w * msg->x + msg->y * msg->z), 1 - 2 * (msg->x*msg->x+ msg->y*msg->y));
        auto pitch   = asin(2 * (msg->w * msg->y - msg->z * msg->x));
        auto yaw     = atan2(2 * (msg->w * msg->z + msg->x * msg->y), 1 - 2 * (msg->y*msg->y + msg->z*msg->z));

        // RCLCPP_INFO(this->get_logger(), "I heard: X : '%f' Y : '%f' Z : '%f", roll,pitch,yaw);
        rotation_count(roll);
        RCLCPP_INFO(this->get_logger(),"ROTATION : '%d'",full_rotations);


        // auto lin_vel_x = msg->angular_velocity.x * wheel_radius ;
        // auto lin_vel_y = msg->angular_velocity.y * wheel_radius ;

        // // auto dis_x = lin_vel_x * time_now;
        // twist.linear.x = lin_vel_x;
        // twist.linear.y = lin_vel_y;e
        // //twist.angular.z = msg->angular_velocity.z;

        // publisher_->publish(twist);


    }


    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr imu_subscriber_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rev_tracker>());
    rclcpp::shutdown();
    return 0;
}