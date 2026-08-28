#include "px4_tf/px4_tf_publisher_node.hpp"
#include "tf2/LinearMath/Quaternion.h"

Px4TfPublisherNode::Px4TfPublisherNode()
    : Node("px4_tf_publisher") {
        px4_tf_prefix_ = this->declare_parameter<std::string>("px4_tf_prefix", "");
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ = 
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        
        // Initialize default continuous transform (identity at origin before odometry arrives)
        current_transform_.header.frame_id = px4_tf_prefix_ + "odom_ned";
        current_transform_.child_frame_id = px4_tf_prefix_ + "base_link_frd";
        current_transform_.transform.translation.x = 0.0;
        current_transform_.transform.translation.y = 0.0;
        current_transform_.transform.translation.z = 0.0;
        current_transform_.transform.rotation.x = 0.0;
        current_transform_.transform.rotation.y = 0.0;
        current_transform_.transform.rotation.z = 0.0;
        current_transform_.transform.rotation.w = 1.0;

        this->make_static_transforms();

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                this->handle_odometry(msg);
            }
        );

        // 20 Hz heartbeat timer to ensure TF tree is always connected
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Px4TfPublisherNode::timer_callback, this)
        );
}

void Px4TfPublisherNode::make_static_transforms() {
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    geometry_msgs::msg::TransformStamped t;

    // odom ENU to odom NED
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = px4_tf_prefix_ + "odom";
    t.child_frame_id = px4_tf_prefix_ + "odom_ned";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI/2);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    static_transforms.push_back(t);

    // base link flu to base link frd
    t.header.frame_id = px4_tf_prefix_ + "base_link_frd";
    t.child_frame_id = px4_tf_prefix_ + "base_link";
    q.setRPY(M_PI, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    static_transforms.push_back(t);

    tf_static_broadcaster_->sendTransform(static_transforms);
}

void Px4TfPublisherNode::handle_odometry(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_transform_.transform.translation.x = msg->position[0];
    current_transform_.transform.translation.y = msg->position[1];
    current_transform_.transform.translation.z = msg->position[2];
    current_transform_.transform.rotation.x = msg->q[1];
    current_transform_.transform.rotation.y = msg->q[2];
    current_transform_.transform.rotation.z = msg->q[3];
    current_transform_.transform.rotation.w = msg->q[0];
    has_odom_ = true;
}

void Px4TfPublisherNode::timer_callback() {
    geometry_msgs::msg::TransformStamped t;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        t = current_transform_;
    }
    t.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(t);
}
