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


typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac;

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay, std::string fixedFrame);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses, ros::NodeHandle nh, std::string fixedFrame)
{
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
  line.header.frame_id = "table_link";
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

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

    if(distance > 0.01)
    {
      Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGTH,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGTH,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);

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
  ROS_INFO_THROTTLE(5,"hi");
  }

  marker_publisher_.publish(markers_msg);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

}

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

  std::string moveGroup, fixedFrame, endEffector, jointTrajectoryCommand;
  if(nh.getParam("/config_data/move_group", moveGroup) &&
     nh.getParam("/config_data/fixed_frame", fixedFrame) &&
     nh.getParam("/config_data/end_effector", endEffector) &&
     nh.getParam("/config_data/joint_trajectory_command", jointTrajectoryCommand) &&
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
  ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ("/" + jointTrajectoryCommand, true);
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
  group.setPoseTarget(target_pose  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  */
  /* Sleep to give Rviz time to visualize the plan. */
  ros::Duration(1.0).sleep();
  group.move();


  TrajectoryVec points;

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
        pose = Eigen::Translation3d((xPos+r*cos(angle*j)), (yPos +(r)*sin(angle*j)), zPos +(o));

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
        descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
        points.push_back(pt);
        poses.push_back(pose);
        ros::Duration(0.1).sleep();
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
  }
  ros::Duration(0.5).sleep();
  publishPosesMarkers(poses, nh, fixedFrame);
  ros::Duration(0.5).sleep();


  /* for (unsigned int i = 0; i < 5; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.0, 0.04 * i, 1.3);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }
  */
  

  // 2. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = moveGroup;

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = fixedFrame;

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = endEffector;

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }
 
  model->setCheckCollisions(true);
/*
    descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joints));
    points.front() = pt;
*/
  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }

  TrajectoryVec result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -3;
  }

  // 5. Translate the result into a type that ROS understands
  // Get Joint Names
  std::vector<std::string> names;
  nh.getParam("/config_data/joint_names", names);
  // Generate a ROS joint trajectory with the result path, robot model, given joint names,
  // a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0, fixedFrame);
  std::vector<double> start_pose = joint_solution.points[0].positions;
  MoveInterface mi = MoveInterface();
  mi.initialize(moveGroup);
  mi.moveJoints(start_pose, 1.0);
  mi.waitForStatus();

  std::vector<double> start_joints = getCurrentJointState("joint_states");
  start_joints.resize(6);
  joint_solution.points[0].positions = start_joints;

  // 6. Send the ROS trajectory to the robot for execution
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -4;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay,
                     std::string fixedFrame)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = fixedFrame;
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  if (!ac->waitForServer(ros::Duration(6.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  ac->sendGoal(goal);

  if (ac->waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}


