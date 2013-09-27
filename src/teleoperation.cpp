#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/move_group_interface/move_group.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>

#define PI 3.14159265

using namespace Eigen;

//https://github.com/ros-planning/moveit_pr2/blob/groovy-devel/pr2_moveit_tutorials/planning/src/planning_scene_tutorial.cpp
//http://moveit.ros.org/wiki/Environment_Representation/C%2B%2B_API#Change_the_state (has errors that are fixed in the above link)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TeleopNode", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Affine3d pose;
  move_group_interface::MoveGroup group("manipulator");
  //Target for the trajectory start
  pose = Translation<double,3>(-0.3, 0.3, 1);
  group.setPoseTarget(pose);
  group.move();

  //ROS_INFO("Moved");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

  planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
  psm.startStateMonitor();
  sleep(1);
  robot_state::RobotState r = psm.getPlanningScene()->getCurrentState();

  /* Get and print the name of the coordinate frame in which the transforms for this model are computed*/
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  std::vector<double> joint_values;  
  robot_state::JointStateGroup* joint_state_group_ref = r.getJointStateGroup("manipulator");
  robot_state::JointStateGroup* joint_state_group_traj = current_state.getJointStateGroup("manipulator");
  joint_state_group_ref->getVariableValues(joint_values);
  joint_state_group_traj->setVariableValues(joint_values);

  /* Get the names of the joints*/
  const std::vector<std::string> &joint_names = joint_state_group_traj->getJointModelGroup()->getJointModelNames();

  /* Print joint names and values */
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint ref %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  Affine3d end_effector_state;
  end_effector_state = Translation<double,3>(0, 0., 1.18);

/* DIFFERENTIAL KINEMATICS */
  /* Get and print the Jacobian for the right arm*/
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;

  //
  // Generate a trajectory
  //

  ros::NodeHandle n;

  ros::Publisher joint_trajectory_action_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 100);
  sleep(1);

  ROS_INFO("Published created");  

  double period = 4;
  VectorXd velVect = VectorXd::Random(6);
  VectorXd artiVel = VectorXd::Random(7);
  std::vector<double> artiVelStd(7);
  double* artiVelStdPtr = &artiVelStd[0];

  trajectory_msgs::JointTrajectory msg;

  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
	msg.joint_names.push_back(joint_names[i]);
  }

  msg.points.resize(1);
  msg.points[0].positions.resize(7);
  msg.points[0].velocities.resize(7);
  msg.points[0].time_from_start = ros::Duration(0);

  bool found_ik;
  double t = 0;
  double t_start = ros::Time::now().toSec();
  int frame_id = 0;

  ros::Rate rate(10);

  while (ros::ok())
  {
	    //Generate cartesian point
		end_effector_state = Translation<double,3>(-0.3*cos(2*PI*t/period), 0.3, 1);
		velVect << 0.3*2*PI*sin(2*PI*t/period)/period, 0, 0, 0, 0, 0;

		found_ik = joint_state_group_traj->setFromIK(end_effector_state, 5, 0.1);
		//ROS_INFO("Finding IK...");
		/* Get and print the joint values */
		if (found_ik)
		{
			joint_state_group_traj->getVariableValues(joint_values);
			
			msg.points[0].positions = joint_values;
			joint_state_group_traj->getJacobian(joint_state_group_traj->getJointModelGroup()->getLinkModelNames().back(),
                                 				reference_point_position,
                                 				jacobian);
			artiVel = jacobian.jacobiSvd(ComputeThinU | ComputeThinV).solve(velVect);
			msg.points[0].velocities.resize(7);
			Map<VectorXd> toArtiVelTraj(artiVelStdPtr, 7);
			msg.points[0].velocities = artiVelStd;
			toArtiVelTraj = artiVel;
		}
		else
		{
			ROS_INFO("Did not find IK solution for %d", frame_id);
		}

		//1 second latency
		msg.header.stamp = ros::Time::now() + ros::Duration(1);
		msg.header.frame_id = frame_id;

		joint_trajectory_action_pub.publish(msg);

		ROS_INFO_STREAM("Message "<<msg.points[0]);
		ROS_DEBUG("Published");

		rate.sleep();

		t+= ros::Time::now().toSec() - t_start;
	    t_start = ros::Time::now().toSec();
		
		frame_id+=1;
  }

  sleep(10);

  ros::waitForShutdown();

  return 0;
}
