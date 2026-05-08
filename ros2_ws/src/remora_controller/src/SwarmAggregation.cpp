// swarm_aggregation_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>

class SwarmAggregationNode : public rclcpp::Node {
public:
    SwarmAggregationNode() : Node("swarm_aggregation")
    {
        this->declare_parameter("robot_namespace", "MainRemora");
        std::string robot_namespace = this->get_parameter("robot_namespace").as_string();
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + robot_namespace + "/cmd_vel", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(robot_namespace + "/pose", 10);
            
        // Subscribers - own pose from Unity. Just to testing Unity connection, but this node is not needed since Unity will publish directly to robot_namespace/pose.
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/" + robot_namespace + "/unity_pose", 10, std::bind(&SwarmAggregationNode::poseCallback, this, std::placeholders::_1));
            
        // Subscriber for obstacles/proximity
        // scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(robot_namespace + "/laserscan", 10, std::bind(&SwarmAggregationNode::scanCallback, this, std::placeholders::_1));
        
        // Subscribe to ALL other robots' poses (this is the swarm magic)
        // for(int i = 0; i < num_robots_; i++) {
        //     if(i != robot_id_) {
        //         auto sub = this->create_subscription<geometry_msgs::msg::Pose>("/robot_" + std::to_string(i) + "/pose", 10, [this, i](geometry_msgs::msg::Pose::SharedPtr msg) {
        //                 neighbor_poses_[i] = *msg;
        //             });
        //         neighbor_subs_.push_back(sub);
        //     }
        // }
        
        // Control timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SwarmAggregationNode::controlLoop, this));
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        my_pose_ = *msg;
        pose_pub_->publish(*msg); // Broadcast to other robots
        RCLCPP_INFO_STREAM(this->get_logger(), "Remora Pose --> Position: [" << msg->position.x << ", " << msg->position.y << "]   ||  Orientation: " << msg->orientation.z);
    }
    
    // void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    //     scan_data_ = *msg;
    // }
    
    void controlLoop() {
        auto cmd = geometry_msgs::msg::Twist();
        
        // AGGREGATION LOGIC
        // if(hasNearbyObstacle()) {
        //     avoidObstacle(cmd);
        // } else if(!neighbor_poses_.empty()) {
        //     moveTowardNeighbors(cmd);
        // } else {
        //     randomWalk(cmd);
        // }
        
        cmd_vel_pub_->publish(cmd);
    }
    
    // bool hasNearbyObstacle() {
    //     if(scan_data_.ranges.empty()) return false;
    //     float min_dist = *std::min_element(scan_data_.ranges.begin(), scan_data_.ranges.end());
    //     return min_dist < 0.5; // 50cm threshold
    // }
    
    // void avoidObstacle(geometry_msgs::msg::Twist& cmd) {
    //     cmd.linear.x = 0.0;
    //     cmd.angular.z = 0.5; // Turn away
    // }
    
    // void moveTowardNeighbors(geometry_msgs::msg::Twist& cmd) {
    //     // Calculate centroid of neighbors
    //     double cx = 0, cy = 0;
    //     int count = 0;
        
    //     for(const auto& [id, pose] : neighbor_poses_) {
    //         cx += pose.pose.position.x;
    //         cy += pose.pose.position.y;
    //         count++;
    //     }
        
    //     if(count == 0) return;
        
    //     cx /= count;
    //     cy /= count;
        
    //     // Vector to centroid
    //     double dx = cx - my_pose_.pose.position.x;
    //     double dy = cy - my_pose_.pose.position.y;
    //     double distance = std::sqrt(dx*dx + dy*dy);
        
    //     // Stop if close enough
    //     if(distance < 1.0) {
    //         cmd.linear.x = 0.0;
    //         cmd.angular.z = 0.0;
    //         return;
    //     }
        
    //     // Calculate heading error
    //     double target_angle = std::atan2(dy, dx);
    //     double current_yaw = getYawFromQuaternion(my_pose_.pose.orientation);
    //     double angle_error = normalizeAngle(target_angle - current_yaw);
        
    //     // Proportional control
    //     cmd.linear.x = std::min(0.5, distance * 0.3);
    //     cmd.angular.z = angle_error * 0.8;
    // }
    
    // void randomWalk(geometry_msgs::msg::Twist& cmd) {
    //     cmd.linear.x = 0.2;
    //     cmd.angular.z = (rand() % 100 - 50) / 100.0;
    // }
    
    // double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    //     return std::atan2(2.0*(q.w*q.z + q.x*q.y), 
    //                      1.0 - 2.0*(q.y*q.y + q.z*q.z));
    // }
    
    // double normalizeAngle(double angle) {
    //     while(angle > M_PI) angle -= 2*M_PI;
    //     while(angle < -M_PI) angle += 2*M_PI;
    //     return angle;
    // }
    
    std::string robot_namespace;
    int robot_id_;
    int num_robots_ = 5;
    
    geometry_msgs::msg::Pose my_pose_;
    sensor_msgs::msg::LaserScan scan_data_;
    std::map<int, geometry_msgs::msg::Pose> neighbor_poses_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> neighbor_subs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmAggregationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}