/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#ifndef _PLACE_GENERATOR_VISUALIZATION_H_
#define _PLACE_GENERATOR_VISUALIZATION_H_

#include <ros/ros.h>
#include <planning_scene/planning_scene.h>
#include <moveit_manipulation_visualization/place_generator_dummy.h>
#include <moveit_manipulation_msgs/Grasp.h>

namespace moveit_manipulation_visualization {

class PlaceGeneratorVisualization {
  
public:
  
  PlaceGeneratorVisualization(ros::Publisher& marker_publisher);

  ~PlaceGeneratorVisualization(){};

  void removeAllMarkers();

  bool generatePlaces(const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const std::string& obj,
                      const std::string& support,
                      std::vector<geometry_msgs::PoseStamped>& place_locations);

  void showPlace(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const std::string& arm_name,
                 const moveit_manipulation_msgs::Grasp& grasp,
                 const geometry_msgs::PoseStamped& place_location);
                 

protected:

  boost::shared_ptr<PlaceGeneratorDummy> place_generator_;

  ros::Publisher marker_publisher_;
  visualization_msgs::MarkerArray last_markers_;

};

}

#endif
