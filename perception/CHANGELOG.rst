^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2016-04-11)
------------------
* [fix] Remove OpenMP parallelization (fixes `#563 <https://github.com/ros-planning/moveit_ros/issues/563>`_)
* Contributors: Stefan Kohlbrecher

0.7.0 (2016-01-30)
------------------
* Removed trailing whitespace from entire repository
* Added missing dependency on moveit_msgs package
* Contributors: Andriy Petlovanyy, Dave Coleman, dg

0.6.5 (2015-01-24)
------------------
* update maintainers
* adding RAII-based locking for OccMapTree
* moving lazy_free_space_updater into it's own library
* Contributors: Jonathan Bohren, Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* port `#445 <https://github.com/ros-planning/moveit_ros/issues/445>`_ to indigo
* disable test that needs display when no display defined
* GL_TYPE() is a function in newer versions of OpenGL, this fixes tests on Ubuntu 14.04
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* fix linking error on OSX
* Contributors: Michael Ferguson

0.6.0 (2014-10-27)
------------------
* Fixing invalid iterators if filtered_cloud_topic is not set.
  Adding missing dependency on sensor_msgs.
  Fixing indentation, whitespace, and tabs.
  Incrementing PointCloud2Iterator pixel-at-a-time, not byte-at-a-time.
* remove PCL dependency
* Fixed issue with unordered_map and libc++ (LLVM, Mac OS X Mavericks)
  libc++ doesn't have std::tr1::unordered_map, just std::unordered_map
* Fixing OpenGL gl.h and glu.h inclusion on Mac OS X
* Contributors: Jason Ziglar, Marco Esposito, Sachin Chitta, Vincent Rabaud

0.5.19 (2014-06-23)
-------------------
* Fix [-Wreorder] warning.
* Address [cppcheck: duplicateExpression] error.
  The existing check for NaNs is in fact correct for IEEE-compliant floating
  numbers, i.e., if (a == a) then a is not a NaN, but confuses static code
  analyzers. This fix instead uses the isnan(a) macro from <cmath>.
* Prevent future conflicts between STL and Boost.
  mesh_filter_base.cpp was doing:
  using namespace std;
  using namespace boost;
  Considering that Boost is a testing ground for future standard additions,
  bringing the two namespaces into scope in the same translation unit is not
  the best idea. In this particular file, there's a potential conflict between
  C++'s and Boost's shared_ptr implementation.
* Make creation of std::pairs future-compiler-proof.
  Details:
  http://stackoverflow.com/questions/14623958/breaking-change-in-c11-with-make-pair-ty1-val1-const-ty2-val2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------
* comply to the new Table.msg
* Contributors: Vincent Rabaud

0.5.9 (2013-12-03)
------------------
* fix cloud offset

0.5.8 (2013-10-11)
------------------
* adds compliance for mesa versions <9.2

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------
* fix `#320 <https://github.com/ros-planning/moveit_ros/issues/320>`_.
* fix `#318 <https://github.com/ros-planning/moveit_ros/issues/318>`_.

0.5.5 (2013-09-23)
------------------
* remove dep on pcl (pcl_conversions is sufficient)

0.5.4 (2013-08-14)
------------------
* add dependency on OpenCV2
* Pointcloud_octomap_updater compilation flags fixed

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------
* find PCL separately

0.5.0 (2013-07-12)
------------------
* use pcl_conversions instead of pcl_ros
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* Fixes linkedit error on OS X
