#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include "move_interface.h"


class dataCollection {
public:
  dataCollection();
  void readPose();
  
private:
  ros::NodeHandle node_;
  MoveInterface moveInterface_;
  const std::string frame_;
};
