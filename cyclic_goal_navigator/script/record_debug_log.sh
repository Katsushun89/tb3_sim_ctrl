#!/bin/bash

ros2 bag record -o ~/bags/cyclic_bug \
  /local_costmap/costmap \
  /tf /tf_static \
  /odom \
  /map \
  /scan \
  /amcl_pose
