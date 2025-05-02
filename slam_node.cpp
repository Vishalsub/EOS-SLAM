#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SlamNode : public rclcpp::Node {
public:
    SlamNode() : Node("slam_node") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SlamNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SlamNode::odom_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received a scan with %zu ranges", msg->ranges.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry at position (%.2f, %.2f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamNode>());
    rclcpp::shutdown();
    return 0;
}

