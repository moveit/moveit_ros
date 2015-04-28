#include <moveit_msgs/PoseTwist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/move_group_interface/move_group.h>
#include "manipulator_state_publisher/manipulator_state_publisher.h"

using namespace Eigen;

namespace manipulator_state
{

ManipulatorStatePublisher::~ManipulatorStatePublisher()
{
  psm_.stopStateMonitor();
}

bool ManipulatorStatePublisher::init()
{
  psm_.startStateMonitor();
  cartesian_pose_twist_ = node_.advertise<lgsm_conversions::PoseTwist>("manipulator_state", 30);
  cs_ = psm_.getStateMonitor();
  if(cs_)
    cs_->addUpdateCallback(boost::bind(&ManipulatorStatePublisher::currentStateCB, this, _1));
  else
  {
    ROS_ERROR("CURRENT STATE POINTER IS EMPTY!!!");
    return false;
  }
  isFirst_ = true;
  return true;
}

void ManipulatorStatePublisher::currentStateCB(const sensor_msgs::JointStateConstPtr& joint_state)
{
  ROS_INFO("Callback");

  robot_state::RobotStatePtr rs = cs_->getCurrentState();
  robot_state::JointStateGroup* joint_state_group = rs->getJointStateGroup(end_effector_name_);

  geometry_msgs::PoseStamped pose_msg;
  geometry_msgs::Twist twist_msg;
  lgsm_conversions::PoseTwist pose_twist_msg;

  Eigen::Matrix<double, 7, 1> joint_values_vector(joint_state->position.data());

  pose_msg = group_.getCurrentPose();

  //Compute the twist from qdot
  joint_state_group->getVariableValues(joint_values_);

  joint_state_group->getJacobian(joint_state_group->getJointModelGroup()->getLinkModelNames().back(),
                                 reference_point_position_,
                                 jacobian_);

  ROS_INFO("Jacobian done");

  if(isFirst_)
  {
    joint_values_vector_old_ = joint_values_vector;
    isFirst_ = false;
  }

  timeDif_ = ros::Time::now() - timestamp_old_;
  timestamp_old_ = ros::Time::now();

  if(timeDif_.toSec() > 0)
  {
    ROS_INFO_STREAM("joint_vales_vector"<<joint_values_vector);
    qdot_values_vector_ = (joint_values_vector - joint_values_vector_old_)/timeDif_.toSec();
    ROS_INFO("qdot recalculated");
  }

  ROS_INFO("Time dif done");

  //ROS_INFO("isfirst: %d", isFirst);

  ROS_INFO_STREAM("qdot: "<<qdot_values_vector_);
  ROS_INFO_STREAM("jacobian: "<<jacobian_);

  //Get the Twist
  twist_ = jacobian_*qdot_values_vector_;

  //ROS_INFO_STREAM("twis_vect:\n"<<twist_vect);

  tf::twistEigenToMsg(twist_, twist_msg);

  //Store the old joint values
  joint_values_vector_old_ = joint_values_vector;

  //Construct the PoseTwist message
  pose_twist_msg.pose = pose_msg.pose;
  pose_twist_msg.twist = twist_msg;

  ROS_INFO_STREAM("pose twist: \n"<<pose_twist_msg);

  cartesian_pose_twist_.publish(pose_twist_msg);
}

}
