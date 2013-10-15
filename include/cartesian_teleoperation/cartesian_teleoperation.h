#ifndef CARTESIAN_TELEOPERATION_H
#define CARTESIAN_TELEOPERATION_H

#include "ros/ros.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <string>
#include <lgsm_conversions/PoseTwist.h>

namespace cartesian_teleoperation
{

class CartesianTeleoperation
{
public:
  CartesianTeleoperation(std::string end_effector_name = "manipulator");
  //~CartesianTeleoperation();

  bool init();
  virtual void run() { ros::spin(); }

protected:
  void streamingCB(const lgsm_conversions::PoseTwistPtr& pose_twist);
  ros::NodeHandle node_;
  ros::Publisher joint_path_command_;
  ros::Subscriber cartesian_streaming_interface_;

  std::string end_effector_name_;
  planning_scene_monitor::PlanningSceneMonitor psm_;

  robot_state::RobotStatePtr command_state_;
  robot_state::RobotStatePtr robot_state_;

  robot_state::JointStateGroup* joint_state_group_robot_;
  robot_state::JointStateGroup* joint_state_group_command_;

  trajectory_msgs::JointTrajectory msg_;
  int seq_;

  Eigen::Affine3d end_effector_pose_;
  Eigen::Matrix<double, 6, 1> end_effector_twist_;

  std::vector<double> joint_velocity_;
  Eigen::Matrix<double, 7, 1> joint_velocity_vector_;

  bool isFirst_;

  Eigen::MatrixXd jacobian_;
  std::vector<double> joint_values_;

  Eigen::Vector3d reference_point_position_;
};

}

#endif
