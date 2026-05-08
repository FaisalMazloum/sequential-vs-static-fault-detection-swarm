#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class RemoraStateEstimator : public rclcpp::Node
{
public:
    RemoraStateEstimator() : Node("boat_state_estimator"), origin_set_(false)
    {
        declare_parameter("robot_namespace", "MainRemora");
        std::string robot_namespace = get_parameter("robot_namespace").as_string();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&RemoraStateEstimator::imuCallback, this, std::placeholders::_1));
        gps_remora_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gnss", 10,std::bind(&RemoraStateEstimator::gpsRemoraCallback, this, std::placeholders::_1));
        gps_base_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/base_station/gnss", 10, std::bind(&RemoraStateEstimator::gpsBaseCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/" + robot_namespace + "/imu", 10, std::bind(&RemoraStateEstimator::imuCallback, this, std::placeholders::_1));
        // gps_remora_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/" + robot_namespace + "/gnss", 10,std::bind(&RemoraStateEstimator::gpsRemoraCallback, this, std::placeholders::_1));
        // gps_base_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gnss/base_station", 10, std::bind(&RemoraStateEstimator::gpsBaseCallback, this, std::placeholders::_1));
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/" + robot_namespace + "/odom", 10);
        
        // Diagnostic timer - warn if origin not set after 5 seconds
        diagnostic_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RemoraStateEstimator::checkOriginSet, this));
        
        RCLCPP_INFO(this->get_logger(), "Remora State Estimator started - waiting for base station");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_orientation_ = msg->orientation;
        has_imu_ = true;
        
        if (has_gps_ && has_imu_ && origin_set_) {
            publishOdometry();
        }
    }
    
    void gpsBaseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (!origin_set_) {
        
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            origin_alt_ = msg->altitude;
            
            // Initialize GeographicLib local cartesian converter
            local_cartesian_.Reset(origin_lat_, origin_lon_, origin_alt_);
            origin_set_ = true;
            
            RCLCPP_INFO(this->get_logger(),
                        "Base station origin set: lat=%.6f, lon=%.6f, alt=%.2f",
                        origin_lat_, origin_lon_, origin_alt_);
            
            // Cancel diagnostic timer once origin is set
            diagnostic_timer_->cancel();
        }
    }
    
    void gpsRemoraCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Guard: Don't process rover GPS until origin is set
        if (!origin_set_) {
            RCLCPP_WARN(this->get_logger(),"Received rover GPS but base station origin not yet set");
            return;
        }
        
        // Convert from geodetic to local cartesian coordinates
        local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, 
                                latest_position_x_, latest_position_y_, latest_position_z_);
        has_gps_ = true;
        
        if (has_gps_ && has_imu_ && origin_set_) {
            publishOdometry();
        }
    }
    
    void checkOriginSet()
    {
        if (!origin_set_) {
            RCLCPP_WARN(this->get_logger(), "Base station origin still not set - check /gnss/base_station topic");
        }
    }
    
    void publishOdometry()
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        // Position from GPS
        odom.pose.pose.position.x = latest_position_x_;
        odom.pose.pose.position.y = latest_position_y_;
        odom.pose.pose.position.z = latest_position_z_;
        
        // Orientation from IMU
        odom.pose.pose.orientation = latest_orientation_;
        
        odom_pub_->publish(odom);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_remora_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_base_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;
    
    // State variables
    bool origin_set_;
    bool has_gps_ = false;
    bool has_imu_ = false;
    double origin_lat_, origin_lon_, origin_alt_;
    double latest_position_x_, latest_position_y_, latest_position_z_;
    geometry_msgs::msg::Quaternion latest_orientation_;
    
    // GeographicLib for coordinate conversion
    GeographicLib::LocalCartesian local_cartesian_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoraStateEstimator>());
    rclcpp::shutdown();
    return 0;
}