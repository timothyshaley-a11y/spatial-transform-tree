#include <iostream>
#include "SpatialTransformTree.hpp"

int main() {
    spatial::TransformTree tf_tree;
    tf_tree.registerFrame("world", "");
    tf_tree.registerFrame("sensor", "world");

    tf_tree.updateTransform("sensor", 
        spatial::Transform{spatial::Vector3{0.0, 1.0, 0.0}, spatial::Quaternion{1.0, 0.0, 0.0, 0.0}}, 100.0);
    tf_tree.updateTransform("sensor", 
        spatial::Transform{spatial::Vector3{1.0, 1.0, 0.0}, spatial::Quaternion{1.0, 0.0, 0.0, 0.0}}, 101.0);

    auto pose = tf_tree.lookupTransform("world", "sensor", 100.5);
    if (pose) {
        std::cout << "Interpolated X position: " << pose->translation.x << std::endl;
    }
    return 0;
}
