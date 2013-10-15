#ifndef MANIPULATOR_STATE_PUBLISHER_H
#define MANIPULATOR_STATE_PUBLISHER_H

#include "ros/ros.h"
#include <string>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <sensor_msgs/JointState.h>
#include <lgsm_conversions/PoseTwist.h>
#include <lgsm_conversions/lgsm_msg.h>

namespace manipulator_state
{

//* The manipulator state publisher class
/**
 * - It adds a callback to the planning scene monitor.
 * - It publishes PoseTwist messages on the manipulator_state topic corresponding to the end_effector's pose and twist.
 */
class ManipulatorStatePublisher
{
public:

  //! The constructor.
  /*!
   * It instanciates everything.
   * \param end_effector_name     One end_effector you defined in the MoveIt! configuration step.
   * \todo Adapt the size to the number of joints (7 joints here)
   */
  ManipulatorStatePublisher(std::string end_effector_name = "manipulator") : end_effector_name_(end_effector_name),
                                                                             psm_("robot_description"),
                                                                             group_(end_effector_name),
                                                                             joint_values_vector_(7),
                                                                             timestamp_old_(ros::Time::now()),
                                                                             isFirst_(true),
                                                                             reference_point_position_(0.0,0.0,0.0),
                                                                             qdot_values_vector_(7)
  {  };
  //! The destructor.
  /*!
   * It stops the planning scene monitor.
   */
  ~ManipulatorStatePublisher();

  //! Initialize the state retrieval.
  /*!
   * It starts the planning scene monitor and binds the callback to the current state monitor.
   */
  bool init();
  //! Run the node.
  virtual void run() { ros::spin(); }

protected:
  //! The current state callback.
  /*!
   * Performs the Forward Kinematics calculation
   * \param JointStateConstPtr which encodes the joint states.
   * \todo consider the case where a joint velocity is provided.
   */
  void currentStateCB(const sensor_msgs::JointStateConstPtr& joint_state);
  ros::NodeHandle node_;
  ros::Publisher cartesian_pose_twist_;
  std::string end_effector_name_;
  robot_state::RobotStatePtr rs_;
  planning_scene_monitor::PlanningSceneMonitor psm_;
  planning_scene_monitor::CurrentStateMonitorPtr cs_;
  move_group_interface::MoveGroup group_;

  std::vector<double> joint_values_;
  Eigen::VectorXd joint_values_vector_;
  Eigen::VectorXd joint_values_vector_old_;
  Eigen::VectorXd qdot_values_vector_;

  ros::Duration timeDif_;
  ros::Time timestamp_old_;

  bool isFirst_;

  Eigen::MatrixXd jacobian_;
  Eigen::Matrix<double, 6, 1> twist_;

  Eigen::Vector3d reference_point_position_;
};

}

#endif
