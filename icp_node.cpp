// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "pcl/point_cloud.h"
// #include "pcl/point_types.h"
// #include <cmath>
// #include <vector>
// #include <Eigen/Dense>

// struct Point2D {
//     float x, y;
// };

// class ICPMatcher : public rclcpp::Node {
// public:
//     ICPMatcher() : Node("icp_matcher") {
//         scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&ICPMatcher::scan_callback, this, std::placeholders::_1));
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     std::vector<Point2D> previous_;
//     bool has_previous_ = false;

//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         std::vector<Point2D> current;
//         for (size_t i = 0; i < msg->ranges.size(); ++i) {
//             float r = msg->ranges[i];
//             if (r < msg->range_min || r > msg->range_max) continue;
//             float angle = msg->angle_min + i * msg->angle_increment;
//             current.push_back({static_cast<float>(r * cos(angle)), static_cast<float>(r * sin(angle))});
//         }

//         if (!has_previous_) {
//             previous_ = current;
//             has_previous_ = true;
//             RCLCPP_INFO(this->get_logger(), "Stored first scan.");
//             return;
//         }

//         // ICP MATCHING
//         auto [R, t] = icp(previous_, current);

//         RCLCPP_INFO(this->get_logger(), "ICP Transform: dx=%.2f dy=%.2f dtheta=%.2f",
//                     t.x(), t.y(), atan2(R(1, 0), R(0, 0)));

//         previous_ = current;
//     }

//     std::pair<Eigen::Matrix2f, Eigen::Vector2f> icp(const std::vector<Point2D>& A, std::vector<Point2D>& B) {
//         const int max_iterations = 10;
//         const float tolerance = 1e-3;
//         Eigen::Matrix2f total_R = Eigen::Matrix2f::Identity();
//         Eigen::Vector2f total_t(0.0f, 0.0f);

//         for (int iter = 0; iter < max_iterations; ++iter) {
//             std::vector<Point2D> matched;
//             for (auto& bp : B) {
//                 float min_dist = 1e9;
//                 Point2D closest;
//                 for (auto& ap : A) {
//                     float dist = std::hypot(ap.x - bp.x, ap.y - bp.y);
//                     if (dist < min_dist) {
//                         min_dist = dist;
//                         closest = ap;
//                     }
//                 }
//                 matched.push_back(closest);
//             }

//             auto [R, t] = compute_transformation(B, matched);
//             apply_transformation(B, R, t);
//             total_t = R * total_t + t;
//             total_R = R * total_R;

//             if (t.norm() < tolerance) break;
//         }
//         return {total_R, total_t};
//     }

//     std::pair<Eigen::Matrix2f, Eigen::Vector2f> compute_transformation(
//         const std::vector<Point2D>& src, const std::vector<Point2D>& tgt) {
//         Point2D cs{0, 0}, ct{0, 0};
//         for (size_t i = 0; i < src.size(); ++i) {
//             cs.x += src[i].x; cs.y += src[i].y;
//             ct.x += tgt[i].x; ct.y += tgt[i].y;
//         }
//         cs.x /= src.size(); cs.y /= src.size();
//         ct.x /= tgt.size(); ct.y /= tgt.size();

//         Eigen::MatrixXf S(2, src.size()), T(2, tgt.size());
//         for (size_t i = 0; i < src.size(); ++i) {
//             S(0,i) = src[i].x - cs.x; S(1,i) = src[i].y - cs.y;
//             T(0,i) = tgt[i].x - ct.x; T(1,i) = tgt[i].y - ct.y;
//         }

//         Eigen::Matrix2f H = S * T.transpose();
//         Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
//         Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
//         Eigen::Vector2f t(ct.x, ct.y);
//         t -= R * Eigen::Vector2f(cs.x, cs.y);
//         return {R, t};
//     }

//     void apply_transformation(std::vector<Point2D>& pts, const Eigen::Matrix2f& R, const Eigen::Vector2f& t) {
//         for (auto& p : pts) {
//             Eigen::Vector2f pt(p.x, p.y);
//             pt = R * pt + t;
//             p.x = pt.x(); p.y = pt.y();
//         }
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ICPMatcher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>

struct Point2D {
    float x, y;
};

class ICPMatcher : public rclcpp::Node {
public:
    ICPMatcher() : Node("icp_matcher"),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
        global_x_(0.0f), global_y_(0.0f), global_theta_(0.0f)
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ICPMatcher::scan_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<Point2D> previous_;
    float global_x_, global_y_, global_theta_;

    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<Point2D> current;
        float angle = msg->angle_min;
        for (auto r : msg->ranges) {
            if (std::isfinite(r)) {
                current.push_back({static_cast<float>(r * std::cos(angle)), static_cast<float>(r * std::sin(angle))});
            }
            angle += msg->angle_increment;
        }

        if (!previous_.empty()) {
            auto [dx, dy, dtheta] = fake_icp(previous_, current);  // Replace with real ICP

            global_x_ += dx * std::cos(global_theta_) - dy * std::sin(global_theta_);
            global_y_ += dx * std::sin(global_theta_) + dy * std::cos(global_theta_);
            global_theta_ += dtheta;

            publish_tf(global_x_, global_y_, global_theta_);

            RCLCPP_INFO(this->get_logger(), "ICP Transform: dx=%.2f dy=%.2f dtheta=%.2f", dx, dy, dtheta);
        }

        previous_ = current;
    }

    void publish_tf(float x, float y, float theta) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);
    }

    std::tuple<float, float, float> fake_icp(const std::vector<Point2D>& ref, const std::vector<Point2D>& cur) {
        // Placeholder: in real ICP, you'd align 'cur' to 'ref' and return dx, dy, dtheta
        return {0.01f, -0.01f, 0.001f};
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPMatcher>());
    rclcpp::shutdown();
    return 0;
}
