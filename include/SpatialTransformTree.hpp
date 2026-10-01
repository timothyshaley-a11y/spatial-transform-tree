/**
 * @file SpatialTransformTree.hpp
 * @brief Thread-Safe, Time-Stamped Dynamic Spatial Transform Tree in C++17
 */

#ifndef SPATIAL_TRANSFORM_TREE_HPP
#define SPATIAL_TRANSFORM_TREE_HPP

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <deque>
#include <shared_mutex>
#include <optional>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace spatial {

struct Vector3 {
    double x{0.0}, y{0.0}, z{0.0};
    Vector3 operator+(const Vector3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vector3 operator-(const Vector3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vector3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
};

struct Quaternion {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};

    Quaternion normalized() const {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm < 1e-12) return {1.0, 0.0, 0.0, 0.0};
        return {w / norm, x / norm, y / norm, z / norm};
    }

    Quaternion operator*(const Quaternion& q) const {
        return Quaternion{
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        }.normalized();
    }

    Vector3 rotate(const Vector3& v) const {
        Quaternion p{0.0, v.x, v.y, v.z};
        Quaternion conjugate{w, -x, -y, -z};
        Quaternion res = (*this) * p * conjugate;
        return {res.x, res.y, res.z};
    }

    Quaternion inverse() const { return {w, -x, -y, -z}; }

    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, double t) {
        Quaternion q1_norm = q1.normalized();
        Quaternion q2_norm = q2.normalized();

        double dot = q1_norm.w * q2_norm.w + q1_norm.x * q2_norm.x + 
                    q1_norm.y * q2_norm.y + q1_norm.z * q2_norm.z;

        if (dot < 0.0) {
            q2_norm = {-q2_norm.w, -q2_norm.x, -q2_norm.y, -q2_norm.z};
            dot = -dot;
        }

        if (dot > 0.9995) {
            Quaternion res{
                q1_norm.w + t * (q2_norm.w - q1_norm.w),
                q1_norm.x + t * (q2_norm.x - q1_norm.x),
                q1_norm.y + t * (q2_norm.y - q1_norm.y),
                q1_norm.z + t * (q2_norm.z - q1_norm.z)
            };
            return res.normalized();
        }

        double theta_0 = std::acos(dot);
        double theta = theta_0 * t;
        double sin_theta = std::sin(theta);
        double sin_theta_0 = std::sin(theta_0);

        double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;

        return Quaternion{
            s0 * q1_norm.w + s1 * q2_norm.w,
            s0 * q1_norm.x + s1 * q2_norm.x,
            s0 * q1_norm.y + s1 * q2_norm.y,
            s0 * q1_norm.z + s1 * q2_norm.z
        }.normalized();
    }
};

struct Transform {
    Vector3 translation;
    Quaternion rotation;

    Transform operator*(const Transform& rhs) const {
        return {
            translation + rotation.rotate(rhs.translation),
            (rotation * rhs.rotation).normalized()
        };
    }

    Transform inverse() const {
        Quaternion inv_rot = rotation.inverse();
        return {
            inv_rot.rotate(Vector3{0.0, 0.0, 0.0} - translation),
            inv_rot
        };
    }
};

struct StampedTransform {
    double timestamp_sec;
    Transform transform;
};

class TransformTree {
public:
    using FrameId = std::string;

    explicit TransformTree(size_t max_history_per_frame = 200)
        : max_history_length_(max_history_per_frame) {}

    void registerFrame(const FrameId& frame_id, const FrameId& parent_id) {
        std::unique_lock<std::shared_mutex> lock(tree_mutex_);
        if (frame_id == parent_id) {
            throw std::invalid_argument("Frame cannot be its own parent: " + frame_id);
        }

        auto node = getOrCreateNodeUnlocked(frame_id);
        if (parent_id.empty()) {
            node->parent = nullptr;
            return;
        }
        auto parent_node = getOrCreateNodeUnlocked(parent_id);
        node->parent = parent_node;
    }

    void updateTransform(const FrameId& child_frame, const Transform& transform, double timestamp_sec) {
        std::unique_lock<std::shared_mutex> lock(tree_mutex_);
        auto node = getOrCreateNodeUnlocked(child_frame);
        auto& history = node->history;
        
        StampedTransform new_sample{timestamp_sec, transform};
        auto it = std::upper_bound(history.begin(), history.end(), timestamp_sec,
            [](double ts, const StampedTransform& elem) { return ts < elem.timestamp_sec; });
        
        history.insert(it, new_sample);
        if (history.size() > max_history_length_) {
            history.pop_front();
        }
    }

    std::optional<Transform> lookupTransform(
        const FrameId& target_frame, 
        const FrameId& source_frame, 
        double timestamp_sec) const 
    {
        std::shared_lock<std::shared_mutex> lock(tree_mutex_);

        if (target_frame == source_frame) {
            return Transform{Vector3{0.0, 0.0, 0.0}, Quaternion{1.0, 0.0, 0.0, 0.0}};
        }

        auto target_node = findNodeUnlocked(target_frame);
        auto source_node = findNodeUnlocked(source_frame);

        if (!target_node || !source_node) return std::nullopt;

        std::vector<std::shared_ptr<FrameNode>> target_chain;
        std::vector<std::shared_ptr<FrameNode>> source_chain;

        for (auto curr = target_node; curr != nullptr; curr = curr->parent) target_chain.push_back(curr);
        for (auto curr = source_node; curr != nullptr; curr = curr->parent) source_chain.push_back(curr);

        std::shared_ptr<FrameNode> lca = nullptr;
        while (!target_chain.empty() && !source_chain.empty() && 
               target_chain.back() == source_chain.back()) {
            lca = target_chain.back();
            target_chain.pop_back();
            source_chain.pop_back();
        }

        if (!lca) return std::nullopt;

        Transform T_lca_source{Vector3{0.0, 0.0, 0.0}, Quaternion{1.0, 0.0, 0.0, 0.0}};
        auto curr_node = source_node;
        for (size_t i = 0; i < source_chain.size(); ++i) {
            auto T_parent_curr = interpolateNodeTransform(curr_node, timestamp_sec);
            if (!T_parent_curr) return std::nullopt;
            T_lca_source = (*T_parent_curr) * T_lca_source;
            curr_node = curr_node->parent;
        }

        Transform T_lca_target{Vector3{0.0, 0.0, 0.0}, Quaternion{1.0, 0.0, 0.0, 0.0}};
        curr_node = target_node;
        for (size_t i = 0; i < target_chain.size(); ++i) {
            auto T_parent_curr = interpolateNodeTransform(curr_node, timestamp_sec);
            if (!T_parent_curr) return std::nullopt;
            T_lca_target = (*T_parent_curr) * T_lca_target;
            curr_node = curr_node->parent;
        }

        return T_lca_target.inverse() * T_lca_source;
    }

private:
    struct FrameNode {
        FrameId id;
        std::shared_ptr<FrameNode> parent{nullptr};
        std::deque<StampedTransform> history;
    };

    mutable std::shared_mutex tree_mutex_;
    std::unordered_map<FrameId, std::shared_ptr<FrameNode>> nodes_;
    size_t max_history_length_;

    std::shared_ptr<FrameNode> findNodeUnlocked(const FrameId& id) const {
        auto it = nodes_.find(id);
        return (it != nodes_.end()) ? it->second : nullptr;
    }

    std::shared_ptr<FrameNode> getOrCreateNodeUnlocked(const FrameId& id) {
        auto node = findNodeUnlocked(id);
        if (!node) {
            node = std::make_shared<FrameNode>();
            node->id = id;
            nodes_[id] = node;
        }
        return node;
    }

    std::optional<Transform> interpolateNodeTransform(
        const std::shared_ptr<FrameNode>& node, 
        double target_time) const 
    {
        const auto& history = node->history;
        if (history.empty()) return std::nullopt;

        if (history.size() == 1 || target_time <= history.front().timestamp_sec) {
            return history.front().transform;
        }
        if (target_time >= history.back().timestamp_sec) {
            return history.back().transform;
        }

        auto it = std::upper_bound(history.begin(), history.end(), target_time,
            [](double ts, const StampedTransform& elem) { return ts < elem.timestamp_sec; });

        const auto& t1_sample = *(it - 1);
        const auto& t2_sample = *it;

        double dt = t2_sample.timestamp_sec - t1_sample.timestamp_sec;
        if (dt <= 1e-9) return t1_sample.transform;

        double alpha = (target_time - t1_sample.timestamp_sec) / dt;

        Vector3 interp_trans = t1_sample.transform.translation + 
            (t2_sample.transform.translation - t1_sample.transform.translation) * alpha;

        Quaternion interp_rot = Quaternion::slerp(
            t1_sample.transform.rotation, 
            t2_sample.transform.rotation, 
            alpha
        );

        return Transform{interp_trans, interp_rot};
    }
};

} // namespace spatial

#endif // SPATIAL_TRANSFORM_TREE_HPP
