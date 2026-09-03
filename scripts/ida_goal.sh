#!/usr/bin/bash

ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 88353.09375, y: 74559.90625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6374472331592352, w: 0.7704940135637885}}}"