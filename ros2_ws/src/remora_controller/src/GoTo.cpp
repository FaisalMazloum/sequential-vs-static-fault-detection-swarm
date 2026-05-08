#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class BoatController : public rclcpp::Node
{
public:
    BoatController() : Node("boat_controller")
    {
        // Declare parameters
        this->declare_parameter("robot_namespace", "MainRemora");

        this->declare_parameter("target_x", 10.0);
        this->declare_parameter("target_y", 10.0);
        this->declare_parameter("goal_tolerance", 1.0);
        this->declare_parameter("k_linear", 50.0);
        this->declare_parameter("k_angular", 8.0);
        this->declare_parameter("max_linear_vel", 200.0);
        this->declare_parameter("max_angular_vel", 5.0);
        
        // Get parameters
        robot_namespace = this->get_parameter("robot_namespace").as_string();

        target_x_ = this->get_parameter("target_x").as_double();
        target_y_ = this->get_parameter("target_y").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        k_linear_ = this->get_parameter("k_linear").as_double();
        k_angular_ = this->get_parameter("k_angular").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,std::bind(&BoatController::odometryCallback, this, std::placeholders::_1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 10);
        goal_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("goal_pose", 10);

        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/" + robot_namespace + "/odom", 10,std::bind(&BoatController::odometryCallback, this, std::placeholders::_1));
        // twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + robot_namespace + "/twist", 10);
        // goal_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/" + robot_namespace + "/goal_pose", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "Boat Controller started. Target: (%.2f, %.2f)", target_x_, target_y_);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract current position
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        
        // Extract current yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        
        
        // Normalize quaternion
        q.normalize();
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw *= -1;
        
        // Check if yaw is valid
        if (std::isnan(yaw) || std::isinf(yaw)) {
            RCLCPP_ERROR(this->get_logger(), "Yaw is NaN or Inf!");
            return;
        }
        
        // Calculate position error
        double dx = target_x_ - current_x;
        double dy = (target_y_ - current_y) * -1; // inverted in unity
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // Calculate desired heading to target
        double desired_yaw = std::atan2(dy, dx) + M_PI / 2; // + 90degrees since there is an offset
        
        // Calculate yaw error (how much we need to turn)
        double yaw_error = normalizeAngle(desired_yaw - yaw);
        
        // EXTENSIVE DEBUG LOGGING
        RCLCPP_INFO(this->get_logger(), 
            "\n=== CONTROLLER DEBUG ===\n"
            "Current pos: (%.2f, %.2f)\n"
            "Target pos:  (%.2f, %.2f)\n"
            "Delta: dx=%.2f, dy=%.2f\n"
            "Distance to goal: %.2f m\n"
            "Current yaw: %.2f rad (%.1f deg)\n"
            "Desired yaw: %.2f rad (%.1f deg)\n"
            "Yaw error:   %.2f rad (%.1f deg)\n"
            "Linear Vel:  (%.2f, %.2f)\n"
            "Angular Vel:  (%.2f, %.2f)\n"
            "=======================",
            current_x, current_y,
            target_x_, target_y_,
            dx, dy,
            distance,
            yaw, yaw * 180.0 / M_PI,
            desired_yaw, desired_yaw * 180.0 / M_PI,
            yaw_error, yaw_error * 180.0 / M_PI,
            linear_vel,
            angular_vel
        );
        
        // Generate control commands
        geometry_msgs::msg::Twist twist_msg;
        // twist.header.stamp = this->now();
        // twist.header.frame_id = "base_link";
        
        if (distance > goal_tolerance_) {
            // Proportional control
            linear_vel = k_linear_ * distance;
            angular_vel = k_angular_ * yaw_error;
            
            // Saturate velocities
            linear_vel = std::clamp(linear_vel, 0.0, max_linear_vel_);
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

            
            twist_msg.linear.x = linear_vel;
            // twist.twist.linear.x = 0.0f;

            twist_msg.angular.z = angular_vel;
            // twist.twist.angular.z = 0.0f;

            
            RCLCPP_INFO(this->get_logger(),
                "Commands: linear_vel=%.2f m/s, angular_vel=%.2f rad/s (%.1f deg/s)\n",
                linear_vel, angular_vel, angular_vel * 180.0 / M_PI);
        } else {
            // Goal reached
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                goal_reached_ = true;
            }
        }
        
        twist_pub_->publish(twist_msg);

        goal_msg.pose.pose.position.x = target_x_;
        goal_msg.pose.pose.position.y = target_y_;
        goal_pub_->publish(goal_msg);
    }
    
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr goal_pub_;
    
    // Parameters
    std::string robot_namespace;
    double target_x_, target_y_;
    double goal_tolerance_;
    double k_linear_, k_angular_;
    double max_linear_vel_, max_angular_vel_;
    double linear_vel, angular_vel;
    nav_msgs::msg::Odometry goal_msg;

    
    // State
    bool goal_reached_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoatController>());
    rclcpp::shutdown();
    return 0;
}