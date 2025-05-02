#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include <opencv2/opencv.hpp>

class MapBuilder : public rclcpp::Node {
public:
    MapBuilder();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void update_map();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    geometry_msgs::msg::Pose2D current_pose_;
    sensor_msgs::msg::LaserScan latest_scan_;

    float resolution_;
    int width_, height_;
    cv::Mat map_;
};
