/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef MANIPULATION_GROUP_H_
#define MANIPULATION_GROUP_H_

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <planning_models/kinematic_model.h>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace pick_place_planner
{

class ManipulationGroup
{

public:
  
  ManipulationGroup(const planning_models::KinematicModelConstPtr &kinematic_model,
                    const std::string &group_name);

  moveit_msgs::AttachedCollisionObject getAttachedBodyMsg(const std::string &body_name, const std::string &arm_name) const;

  moveit_msgs::AttachedCollisionObject getAttachedBodyMsg(const std::string &body_name) const;
  
private:  

  std::string group_name_;
  
  std::vector<std::string> arm_names_;
  
  std::map<std::string, std::vector<std::string> > arm_link_names_map_;

  std::map<std::string,std::string> end_effector_names_map_;

  std::map<std::string, std::vector<std::string> > end_effector_link_names_map_;

  std::vector<std::string> end_effector_link_names_;
  
  planning_models::KinematicModelConstPtr kinematic_model_;  

};

typedef boost::shared_ptr<ManipulationGroup> ManipulationGroupPtr;
typedef boost::shared_ptr<const ManipulationGroup> ManipulationGroupConstPtr;

}

#endif
