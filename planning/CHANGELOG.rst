^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.6 (2013-07-18)
------------------
* add docs for planning pipeline
* white space fixes (tabs are now spaces)
* adding check for approximate solution flag
* adding options struct to kinematics base
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Namespaced ROS_* log messages for better debug fitering - added 'kdl' namespace
* remove dep
* make searchPositionIK actually const, and thread-safe
* Made debug output look better

0.4.4 (2013-06-26)
------------------
* fix `#210 <https://github.com/ros-planning/moveit_ros/issues/210>`_
* added dynamic reconfigure parameters to allow enabling/disabling of trajectory duration monitoring. fixes `#256 <https://github.com/ros-planning/moveit_ros/issues/256>`_
* add state operations evaluation tool
* warn when time parametrization fails
* moved exceptions headers
