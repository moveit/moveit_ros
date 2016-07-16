/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Robert Haschke */

#include "execute_action_capability.h"
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/move_group/capability_names.h>

using namespace trajectory_execution_manager;

namespace move_group {

MoveGroupExecuteAction::MoveGroupExecuteAction():
  MoveGroupCapability("ExecuteTrajectoryAction")
{
}

void MoveGroupExecuteAction::initialize()
{
  // start the move action server
  action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction>
                       (root_node_handle_, EXECUTE_ACTION_NAME,
                        boost::bind(&MoveGroupExecuteAction::executeCallback, this, _1), false));
  action_server_->registerPreemptCallback(boost::bind(&MoveGroupExecuteAction::preemptCallback, this));
  action_server_->start();
}

void MoveGroupExecuteAction::executeCallback(const moveit_msgs::ExecuteTrajectoryGoalConstPtr& goal)
{
  ROS_INFO("Received new trajectory execution request...");
  moveit_msgs::ExecuteTrajectoryResult res;
  if (!context_->trajectory_execution_manager_)
  {
    std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    action_server_->setAborted(res, response);
  }

  // interrupt any currently executed trajectory
  context_->trajectory_execution_manager_->stopExecution(true);
  if (!context_->trajectory_execution_manager_->push(goal->trajectory))
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    action_server_->setAborted(res, "Failed to push trajectory for execution");
  }

  context_->trajectory_execution_manager_->execute(TrajectoryExecutionManager::ExecutionCompleteCallback(),
                                                   boost::bind(&MoveGroupExecuteAction::pathCompleteCallback, this, _1));
  moveit_controller_manager::ExecutionStatus es = context_->trajectory_execution_manager_->waitForExecution();
  ROS_INFO_STREAM("Execution completed: " << es.asString());

  if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
  else if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
  else res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  action_server_->setSucceeded(res);
}

void MoveGroupExecuteAction::preemptCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

void MoveGroupExecuteAction::pathCompleteCallback(std::size_t n)
{
  moveit_msgs::ExecuteTrajectoryFeedback feedback;
  feedback.segments_completed = n;
  action_server_->publishFeedback(feedback);
}

}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteAction, move_group::MoveGroupCapability)
