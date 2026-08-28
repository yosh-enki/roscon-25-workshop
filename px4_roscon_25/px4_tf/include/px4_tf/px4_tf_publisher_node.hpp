#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

class Px4TfPublisherNode : public rclcpp::Node {
public:
    Px4TfPublisherNode();
private:
    std::string px4_tf_prefix_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped current_transform_;
    std::mutex mutex_;
    bool has_odom_{false};

    void make_static_transforms();
    void handle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void timer_callback();
};