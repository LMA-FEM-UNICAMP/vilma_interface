#!/usr/bin/bash

ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 88172.34375, y: 75081.2109375, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.6690139345682089, w: 0.743249860648197}}}"
