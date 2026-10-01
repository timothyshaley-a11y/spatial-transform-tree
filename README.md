# Spatial Transform Tree

A high-performance C++17 thread-safe spatial transform tree with dynamic frame registration, SLERP quaternion interpolation, ROS 2 bridge, and UDP/WebRTC binary streaming protocol.

## Features
- **Header-only Core**: Thread-safe dynamic DAG pose propagation with SLERP & LERP temporal interpolation.
- **ROS 2 Integration**: Native translation between `geometry_msgs::msg::TransformStamped` and tree structures.
- **Binary Network Protocol (BSPS-v1)**: C11 wire-format parser with clock-drift compensation.
