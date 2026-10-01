/**
 * @file Ros2TransformBridge.hpp
 * @brief Bidirectional C++ ROS 2 Bridge for spatial::TransformTree
 */

#ifndef ROS2_TRANSFORM_BRIDGE_HPP
#define ROS2_TRANSFORM_BRIDGE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "SpatialTransformTree.hpp"

namespace spatial::bridge {

inline spatial::Transform rosToSpatialTransform(const geometry_msgs::msg::Transform& ros_tf) {
    return spatial::Transform{
        spatial::Vector3{ros_tf.translation.x, ros_tf.translation.y, ros_tf.translation.z},
        spatial::Quaternion{ros_tf.rotation.w, ros_tf.rotation.x, ros_tf.rotation.y, ros_tf.rotation.z}
    };
}

inline geometry_msgs::msg::Transform spatialToRosTransform(const spatial::Transform& sp_tf) {
    geometry_msgs::msg::Transform ros_tf;
    ros_tf.translation.x = sp_tf.translation.x;
    ros_tf.translation.y = sp_tf.translation.y;
    ros_tf.translation.z = sp_tf.translation.z;
    ros_tf.rotation.w = sp_tf.rotation.w;
    ros_tf.rotation.x = sp_tf.rotation.x;
    ros_tf.rotation.y = sp_tf.rotation.y;
    ros_tf.rotation.z = sp_tf.rotation.z;
    return ros_tf;
}

class Ros2TransformBridge : public rclcpp::Node {
public:
    explicit Ros2TransformBridge(
        std::shared_ptr<spatial::TransformTree> spatial_tree,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("ros2_transform_bridge", options), spatial_tree_(spatial_tree) 
    {
        if (!spatial_tree_) {
            spatial_tree_ = std::make_shared<spatial::TransformTree>();
        }

        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::SystemDefaultsQoS(),
            std::bind(&Ros2TransformBridge::handleTfMessage, this, std::placeholders::_1));

        tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable(),
            std::bind(&Ros2TransformBridge::handleTfMessage, this, std::placeholders::_1));

        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_out", 10);
    }

    void ingestRosTransform(const geometry_msgs::msg::TransformStamped& msg) {
        std::string child_frame = stripLeadingSlash(msg.child_frame_id);
        std::string parent_frame = stripLeadingSlash(msg.header.frame_id);

        double timestamp_sec = static_cast<double>(msg.header.stamp.sec) +
                               (static_cast<double>(msg.header.stamp.nanosec) * 1e-9);

        spatial_tree_->registerFrame(child_frame, parent_frame);
        spatial_tree_->updateTransform(child_frame, rosToSpatialTransform(msg.transform), timestamp_sec);
    }

    std::optional<geometry_msgs::msg::TransformStamped> exportRosTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& query_time) 
    {
        double timestamp_sec = query_time.seconds();
        auto sp_tf = spatial_tree_->lookupTransform(target_frame, source_frame, timestamp_sec);
        if (!sp_tf) return std::nullopt;

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = query_time;
        msg.header.frame_id = target_frame;
        msg.child_frame_id = source_frame;
        msg.transform = spatialToRosTransform(*sp_tf);
        return msg;
    }

    void publishBatchTransforms(
        const std::vector<std::pair<std::string, std::string>>& frame_pairs,
        const rclcpp::Time& query_time) 
    {
        tf2_msgs::msg::TFMessage tf_msg;
        for (const auto& [target_frame, source_frame] : frame_pairs) {
            auto ts = exportRosTransform(target_frame, source_frame, query_time);
            if (ts) tf_msg.transforms.push_back(*ts);
        }
        if (!tf_msg.transforms.empty()) tf_pub_->publish(tf_msg);
    }

private:
    std::shared_ptr<spatial::TransformTree> spatial_tree_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    void handleTfMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto& transform : msg->transforms) {
            ingestRosTransform(transform);
        }
    }

    static std::string stripLeadingSlash(const std::string& frame_id) {
        if (!frame_id.empty() && frame_id[0] == '/') return frame_id.substr(1);
        return frame_id;
    }
};

} // namespace spatial::bridge

#endif // ROS2_TRANSFORM_BRIDGE_HPP
