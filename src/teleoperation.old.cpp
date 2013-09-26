#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <Eigen/Geometry>

using namespace Eigen;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TeleopNode", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // this connecs to a running instance of the move_group node
  move_group_interface::MoveGroup group("manipulator");
  // specify that our target will be a random one
  //group.setRandomTarget();
  Affine3d pose1, pose2;
  pose1 = Translation<double,3>(0, 0., 1.18);
  pose2 = Translation<double,3>(0.1,0.1,0.7);
  group.setPoseTarget(pose1);
  // plan the motion and then move the group to the sampled target 
  group.move();
  ros::Duration(2).sleep();
  group.setPoseTarget(pose2);
  group.move();
  ros::Duration(2).sleep();
  group.setPoseTarget(pose1);
  group.move();

  ros::Duration(2).sleep();
  group.setPoseTarget(pose2);
  group.move();

  ros::waitForShutdown();
}
