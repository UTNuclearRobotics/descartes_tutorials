// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include "move_interface/move_interface.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <string>
#include <vector>


std::vector<double> getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
  if (!state) throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (2);
  spinner.start();

  float xPos;
  float yPos;
  float zPos;

  std::string moveGroup, fixedFrame;
  if(nh.getParam("/config_data/move_group", moveGroup) &&
     nh.getParam("/config_data/fixed_frame", fixedFrame) &&
     nh.getParam("/config_data/x_pos", xPos) &&
     nh.getParam("/config_data/y_pos", yPos) &&
     nh.getParam("/config_data/z_pos", zPos))
  {
    
  }
  else
  {
    ROS_ERROR("Unable to get config data from param server. Ending demo.");
    return false;
  }
  moveit::planning_interface::MoveGroupInterface group(moveGroup); 
  EigenSTL::vector_Affine3d poses;
  // 1. Define sequence of points  
  /* geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.0;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.5;
  Eigen::Quaterniond quat;
  Eigen::Quaterniond quat1;
  double rotationRadians = 3.14195/2;
  Eigen::Vector3d rotationVector = Eigen::Vector3d(0,0,1);
  quat = Eigen::AngleAxisd(rotationRadians, rotationVector);
  rotationVector = Eigen::Vector3d(0,1,0);
  quat1 = Eigen::AngleAxisd(rotationRadians, rotationVector);
  quat = quat*quat1;
  tf::quaternionEigenToMsg(quat, target_pose1.orientation);
  group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  */
  /* Sleep to give Rviz time to visualize the plan. */
  ros::Duration(1.0).sleep();

  double R = 0.04;
  double angle = 2*3.14195/21;
  double offset = (R-0.005)/3;
 
 for (unsigned int i = 0; i < 1; ++i)
  {
    Eigen::Affine3d pose;
    double o = offset*i;
    double r = sqrt(R*R-o*o);
    for (unsigned int j = 0; j < 20; j++) // j = 21
    {
        pose = Eigen::Translation3d((xPos+r*cos(angle*j)), (yPos+(r)*sin(angle*j)), zPos +(o));

        Eigen::Quaterniond quat;
        double rotationRadians = 3.14195;
        Eigen::Vector3d rotationVector = Eigen::Vector3d(0,1,0);
        quat = Eigen::AngleAxisd(rotationRadians, rotationVector);
        Eigen::Matrix3d rotMat; rotMat = quat;
        pose.linear() *= rotMat;
/*
        rotationRadians = 3.14195/2;
        rotationVector = Eigen::Vector3d(0,1,0);
        quat = Eigen::AngleAxisd(rotationRadians, rotationVector);
        rotMat = quat;
        pose.linear() *= rotMat;
        rotationRadians = 3.14195/2;
        rotationVector = Eigen::Vector3d(0,1,0);
        quat = Eigen::AngleAxisd(rotationRadians, rotationVector);
        rotMat = quat;
        pose.linear() *= rotMat;
*/
        poses.push_back(pose);
        ros::Duration(0.1).sleep();
    }
  }
    /*
    Eigen::Quaterniond quat;
    double rotationRadians = 3.14195/2;
    Eigen::Vector3d rotationVector = Eigen::Vector3d(0,0,1);
    quat = Eigen::AngleAxisd(rotationRadians, rotationVector);
    Eigen::Matrix3d rotMat; rotMat = quat;
    pose.linear() *= rotMat;
    rotationRadians = 3.14195/2;
    rotationVector = Eigen::Vector3d(0,1,0);
    quat = Eigen::AngleAxisd(rotationRadians, rotationVector);
    rotMat = quat;
    pose.linear() *= rotMat;
    */
    /*
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
    poses.push_back(pose);
    */
  ros::Duration(0.1).sleep();
  ros::Duration(0.5).sleep();
  

  MoveInterface mi = MoveInterface();
  mi.initialize(moveGroup);

  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;
  float AXIS_LINE_WIDTH = 0.02;
  float AXIS_LINE_LENGTH = 0.02;
  ros::Publisher marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("chatter", 1000);
  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = fixedFrame;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration(0);
  line.header.frame_id = fixedFrame;
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 0;
  line.color.b = 0;
  line.color.a = 0;

  // creating axes markers
  z_axes.points.reserve(2*poses.size());
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  double distance = 0;
  Eigen::Affine3d prev = poses[0];
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];
    distance = (pose.translation() - prev.translation()).norm();

    tf::pointEigenToMsg(pose.translation(),p_start);
    geometry_msgs::Pose pose_request;
    tf::poseEigenToMsg(pose,pose_request);
    geometry_msgs::PoseStamped pose_test;
    pose_test.header.frame_id = fixedFrame;
    pose_test.pose = pose_request;
    bool success = mi.planMove(pose_test, 1.0, false);
    // mi.moveArm(pose_test, 1.0, false);
    if(success==0)
    {
      Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGTH,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);
    }
    else
    {
      Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGTH,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);
    }
    if(distance > 0.01)
    {

      Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGTH);
      tf::pointEigenToMsg(moved_along_z.translation(),p_end);
      z_axes.points.push_back(p_start);
      z_axes.points.push_back(p_end);

      // saving previous
      prev = pose;
    }

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  while(marker_publisher_.getNumSubscribers() < 1)
  {
  ros::Duration(1.0).sleep();
  ROS_INFO_THROTTLE(5,"Check RViz to make sure you are subscribed to the marker array /chatter.");
  }
  
  marker_publisher_.publish(markers_msg);
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::Duration(0.5).sleep();


  /* for (unsigned int i = 0; i < 5; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.0, 0.04 * i, 1.3);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }
  */

  // Wait till user kills the process (Control-C)
  return 0;
}
