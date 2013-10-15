#include "cartesian_teleoperation/cartesian_teleoperation.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/move_group_interface/move_group.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace Eigen;

namespace cartesian_teleoperation
{

CartesianTeleoperation::CartesianTeleoperation(std::string end_effector_name): end_effector_name_(end_effector_name),
                                                                               psm_("robot_description"),
                                                                               isFirst_(true),
                                                                               reference_point_position_(0.0,0.0,0.0),
                                                                               seq_(0),
                                                                               joint_velocity_(7)
{
  cartesian_streaming_interface_ = node_.subscribe("cartesian_streaming_interface",
                                                   100,
                                                   &CartesianTeleoperation::streamingCB, this);

  //Get the real robot's current state
  psm_.startStateMonitor();
  robot_state_ = psm_.getStateMonitor()->getCurrentState();

  //Instantiate a new robot representation from the robot_descritpion. The planning_scene might be an overkill.
  //TODO: add exceptions as in pr2_moveit_tutorials/interactivity/src/interactive_robot.cpp
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  command_state_.reset(new robot_state::RobotState(kinematic_model));
  command_state_->setToDefaultValues();

  //Copy the current state of the real robot to the new robot representation
  std::vector<double> joint_values;
  joint_state_group_robot_ = robot_state_->getJointStateGroup(end_effector_name_);
  joint_state_group_command_ = command_state_->getJointStateGroup(end_effector_name_);
  joint_state_group_robot_->getVariableValues(joint_values);
  joint_state_group_command_->setVariableValues(joint_values);

  //Message skeleton generation
  std::vector<std::string> joint_names = joint_state_group_command_->getJointModelGroup()->getJointModelNames();
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    msg_.joint_names.push_back(joint_names[i]);
  }
  msg_.points.resize(1);
  msg_.points[0].positions.resize(joint_names.size());
  msg_.points[0].positions = joint_values;
  msg_.points[0].velocities.resize(joint_names.size());
  msg_.points[0].time_from_start = ros::Duration(0);
}

bool CartesianTeleoperation::init()
{
  Affine3d pose;
  move_group_interface::MoveGroup group("manipulator");
  //Target for the trajectory start
  pose = Translation<double,3>(-0.3, 0.3, 1);
  group.setPoseTarget(pose);
  group.move();

  //Now we can publish commands
  joint_path_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 100);
  return true;
}

void CartesianTeleoperation::streamingCB(const lgsm_conversions::PoseTwistPtr& pose_twist)
{
  ROS_DEBUG("Callback");
  if(isFirst_)
  {
    msg_.header.stamp = ros::Time::now();
    isFirst_ = false;
  }
  bool found_ik;

  ROS_INFO("Callback");
  //Generate cartesian point
  tf::poseMsgToEigen(pose_twist->pose, end_effector_pose_);
  tf::twistMsgToEigen(pose_twist->twist,end_effector_twist_);

  found_ik = joint_state_group_command_->setFromIK(end_effector_pose_, 5, 0.1);
  /* Get and print the joint values */
  if (found_ik && seq_>0)
  {
    joint_state_group_command_->getVariableValues(joint_values_);
    msg_.points[0].positions = joint_values_;

    //Voir aussi computeJointVelocity
    joint_state_group_command_->getJacobian(joint_state_group_command_->getJointModelGroup()
                                                                      ->getLinkModelNames().back(),
                                            reference_point_position_,
                                            jacobian_
                                            );
    joint_velocity_vector_ = jacobian_.jacobiSvd(ComputeThinU | ComputeThinV).solve(end_effector_twist_);
    Matrix<double, 7, 1>::Map(&joint_velocity_[0], 7, 1) = joint_velocity_vector_;
    msg_.points[0].velocities = joint_velocity_;
  }
  else
  {
    ROS_INFO("Did not find IK solution for %d", seq_);
  }

  //1 second latency
  //msg_.header.stamp = ros::Time::now() + ros::Duration(1);

  msg_.header.seq = seq_;
  msg_.points[0].time_from_start = ros::Time::now() - msg_.header.stamp;
  joint_path_command_.publish(msg_);

  ROS_DEBUG("Published");

  seq_+=1;
}

}
