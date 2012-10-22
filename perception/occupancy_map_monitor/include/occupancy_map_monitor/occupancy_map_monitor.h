/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney */

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_H_
#define MOVEIT_OCCUPANCY_MAP_MONITOR_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <occupancy_map_monitor/occupancy_map.h>
#include <occupancy_map_monitor/occupancy_map_updater.h>
#include <set>

namespace occupancy_map_monitor
{

class OccupancyMapMonitor
{
public:
  
  struct Options
  {
    Options(void) : map_resolution(0.0)
    {
    }
    
    std::string map_frame;
    double map_resolution;
  };
  
  OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer> &tf); 
  OccupancyMapMonitor(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf);

  ~OccupancyMapMonitor(void);
  
  /** @brief start the monitor (will begin updating the octomap */
  void startMonitor(void);
  
  void stopMonitor(void);
  
  /** @brief Get a pointer to the underlying octree for this monitor. Lock the tree before reading or writing using this
   *  pointer. The value od this pointer stays the same throughout the existance of the monitor instance. */
  OccMapTreePtr getOcTreePtr(void)
  {
    return tree_;
  }

  /** @brief Get a const pointer to the underlying octree for this monitor. Lock the
   *  tree before reading this pointer */
  OccMapTreeConstPtr getOcTreePtr(void) const
  {
    return tree_const_;
  }
    
  /** @brief lock the underlying octree. it will not be read or written by the
   *  monitor until unlockTree() is called */
  void lockOcTree(void);
  
  /** @brief unlock the underlying octree. */
  void unlockOcTree(void);
  
  /** @brief Set the callback to trigger when updates to the maintained octomap are received */
  void setUpdateCallback(const boost::function<void(void)> &update_callback)
  {
    update_callback_ = update_callback;
  }

private:

  void initialize(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf);

  /** @brief tells the server an update is ready */
  void updateReady(OccupancyMapUpdater *updater);
  
  void treeUpdateThread(void);
  void publish_markers(void);
  void publish_octomap_binary(void);

  Options opt_;
  
  OccMapTreePtr tree_;
  OccMapTreeConstPtr tree_const_;
  boost::mutex tree_mutex_;
  boost::scoped_ptr<boost::thread> tree_update_thread_;
  bool tree_update_thread_running_;

  std::vector<boost::shared_ptr<OccupancyMapUpdater> > map_updaters_;
  std::set<OccupancyMapUpdater*> updates_available_;
  
  boost::condition_variable update_cond_;
  boost::mutex update_mut_;
  boost::function<void(void)> update_callback_;
  
  ros::NodeHandle root_nh_;
  ros::NodeHandle nh_;
  ros::Publisher occupied_marker_pub_;
  ros::Publisher free_marker_pub_;
  ros::Publisher octree_binary_pub_;
};

}

#endif /* MOVEIT_OCCUPANCY_MAP_MONITOR_H_ */
