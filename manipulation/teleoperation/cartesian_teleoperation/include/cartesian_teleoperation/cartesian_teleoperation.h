#ifndef CARTESIAN_TELEOPERATION_H
#define CARTESIAN_TELEOPERATION_H

#include "ros/ros.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <string>
#include <moveit_msgs/PoseTwist.h>

namespace cartesian_teleoperation
{
//* The cartesian teleoperation class
/**
 * - It subscribes to PoseTwist messages from the cartesian_streaming_interface topic.
 * - It publishes trajectory_msgs::JointTrajectory messages _with only one point_ to joint_path_command.
 *
 */
class CartesianTeleoperation
{
public:
  //! The constructor.
  /*!
   * It instanciates everything, starts the planning scene monitor, initializes a second description of
   * the robot used for IK, and initializes the JointTrajectory messages used afterwards.
   * \param end_effector_name     One end_effector you defined in the MoveIt! configuration step.
   */
  CartesianTeleoperation(std::string end_effector_name = "manipulator");
  //! The destructor.
  /*!
   * It stops the planning scene monitor.
   */
  ~CartesianTeleoperation();

  //! Initialize the teleoperation.
  /*!
   * It moves the robot to a starting position and initializes the publisher.
   * \todo add a parameter to set the starting position dynamically.
   */
  bool init();
  //! Run the node.
  virtual void run() { ros::spin(); }

protected:
  //! The streaming callback.
  /*!
   * Performs the Inverse Kinematics calculation
   * \param pose_twist encodes a Pose and a Twist (see geometry_msgs).
   * \todo be more flexible as to what input can be used (ie Pose only).
   */
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
