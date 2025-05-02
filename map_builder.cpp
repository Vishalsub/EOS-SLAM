#include "my_slam/map_builder.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

namespace {

// Convert world coordinates (meters) to map grid cell indices
cv::Point2i worldToMap(float x, float y, float resolution, int width, int height) {
    int mx = static_cast<int>(x / resolution + width / 2);
    int my = static_cast<int>(height / 2 - y / resolution);
    return cv::Point2i(mx, my);
}

} // namespace

MapBuilder::MapBuilder()
: Node("map_builder"),
  resolution_(0.05), width_(400), height_(400) {

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MapBuilder::scan_callback, this, _1));

    pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
        "/icp_pose", 10, std::bind(&MapBuilder::pose_callback, this, _1));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    map_ = cv::Mat::zeros(height_, width_, CV_8UC1);
}

void MapBuilder::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
    update_map();
}

void MapBuilder::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    current_pose_ = *msg;
}

void MapBuilder::update_map() {
    if (latest_scan_.ranges.empty()) return;

    float angle = latest_scan_.angle_min;
    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
        float r = latest_scan_.ranges[i];
        if (r < latest_scan_.range_min || r > latest_scan_.range_max) {
            angle += latest_scan_.angle_increment;
            continue;
        }

        float wx = current_pose_.x + r * std::cos(current_pose_.theta + angle);
        float wy = current_pose_.y + r * std::sin(current_pose_.theta + angle);

        cv::Point2i start = worldToMap(current_pose_.x, current_pose_.y, resolution_, width_, height_);
        cv::Point2i end = worldToMap(wx, wy, resolution_, width_, height_);

        // Free space
        cv::LineIterator it(map_, start, end);
        for (int j = 0; j < it.count - 1; j++, ++it) {
            map_.at<uchar>(it.pos()) = std::max(0, map_.at<uchar>(it.pos()) - 10);
        }

        // Hit cell
        if (end.x >= 0 && end.x < width_ && end.y >= 0 && end.y < height_)
            map_.at<uchar>(end) = std::min(255, map_.at<uchar>(end) + 50);

        angle += latest_scan_.angle_increment;
    }

    // Convert to OccupancyGrid
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = "map";
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    grid.info.origin.position.x = -width_ * resolution_ / 2;
    grid.info.origin.position.y = -height_ * resolution_ / 2;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(width_ * height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            uint8_t val = map_.at<uchar>(y, x);
            grid.data[y * width_ + x] = (val == 0 ? -1 : val * 100 / 255);
        }
    }

    map_pub_->publish(grid);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapBuilder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
