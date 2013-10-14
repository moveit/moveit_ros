#include <ros/ros.h>
#include <lgsm_conversions/PoseTwist.h>
#include <lgsm_conversions/lgsm_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/move_group_interface/move_group.h>

using namespace Eigen;

ros::NodeHandle node;

ros::Publisher cartesian_pose_twist = node.advertise<lgsm_conversions::PoseTwist>("manipulator_state", 30);

void currentStateCB(planning_scene_monitor::CurrentStateMonitor* cs, const sensor_msgs::JointStateConstPtr &)
{
  robot_state::RobotStatePtr rs = cs->getCurrentState();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manipulator_state_publisher");

  //Essential!
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_group_interface::MoveGroup group("manipulator");
  planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
  psm.startStateMonitor();
  sleep(1);

  robot_state::RobotState r = psm.getPlanningScene()->getCurrentState();
  robot_state::JointStateGroup* joint_state_group = r.getJointStateGroup("manipulator");
  std::vector<double> joint_values(7);

  Map<VectorXd> joint_values_vector(&joint_values[0], 7);
  VectorXd joint_values_vector_old;
  VectorXd qdot_values_vector(7);

  Twistd twist;
  ros::Duration timeDif;
  ros::Time timestamp = ros::Time::now();
  ros::Time timestamp_old = ros::Time::now();

  bool isFirst = true;

  Eigen::MatrixXd jacobian;
  VectorXd twist_vect;

  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);

  ros::Rate rate(100);
  while(node.ok())
  {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::Twist twist_msg;
    lgsm_conversions::PoseTwist pose_twist_msg;

    pose_msg = group.getCurrentPose();

    timestamp = pose_msg.header.stamp;

    //ROS_INFO_STREAM("Current Pose: "<<pose_msg);

    if(timestamp > timestamp_old)
    {
      r =  psm.getPlanningScene()->getCurrentState();
      joint_state_group = r.getJointStateGroup("manipulator");

      joint_state_group->getVariableValues(joint_values);

      joint_state_group->getJacobian(joint_state_group->getJointModelGroup()->getLinkModelNames().back(),
                                     reference_point_position,
                                     jacobian);
      if(isFirst)
      {
        joint_values_vector_old = joint_values_vector;
        isFirst = false;
      }

      //ROS_INFO_STREAM("Joint values: "<<joint_values_vector);
      timeDif = timestamp - timestamp_old;
      timestamp_old = timestamp;

      if(timeDif.toSec() > 0)
        qdot_values_vector = (joint_values_vector - joint_values_vector_old)/timeDif.toSec();

      //ROS_INFO("isfirst: %d", isFirst);

      /*if(qdot_values_vector[0] == 0)
          ROS_INFO("QDOT NULL");
      else
          ROS_INFO_STREAM("qdot: "<<qdot_values_vector);*/

      twist_vect = jacobian*qdot_values_vector;
      twist.get() = twist_vect;

      //ROS_INFO_STREAM("twis_vect:\n"<<twist_vect);

      tf::twistEigenToMsg(twist, twist_msg);

      joint_values_vector_old = joint_values_vector;

      pose_twist_msg.pose = pose_msg.pose;
      pose_twist_msg.twist = twist_msg;

      ROS_INFO_STREAM("pose twist: \n"<<pose_twist_msg);

      cartesian_pose_twist.publish(pose_twist_msg);
    }
  }
  return 0;
}
