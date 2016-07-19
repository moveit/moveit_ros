/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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

/* Author: Kentaro Wada */

#include "execute_action_capability.h"

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupExecuteAction::MoveGroupExecuteAction() :
  MoveGroupCapability("ExecutePathAction"),
  execute_state_(IDLE)
{
}

void move_group::MoveGroupExecuteAction::initialize()
{
  // start the move action server
  execute_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::ExecuteKnownTrajectoryAction>(root_node_handle_, move_group::EXECUTE_ACTION,
                                                                                             boost::bind(&MoveGroupExecuteAction::executeMoveCallback, this, _1), false));
  execute_action_server_->registerPreemptCallback(boost::bind(&MoveGroupExecuteAction::preemptMoveCallback, this));
  execute_action_server_->start();
}

void move_group::MoveGroupExecuteAction::executeMoveCallback(const moveit_msgs::ExecuteKnownTrajectoryGoalConstPtr& goal)
{
  moveit_msgs::ExecuteKnownTrajectoryResult action_res;
  executeMoveCallback_Execute(goal, action_res);

  std::string response = getActionResultString(action_res.error_code, false, false);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    execute_action_server_->setSucceeded(action_res, response);
  }
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    {
      execute_action_server_->setPreempted(action_res, response);
    }
    else
    {
      execute_action_server_->setAborted(action_res, response);
    }
  }

  setExecuteState(IDLE);
}

void move_group::MoveGroupExecuteAction::executeMoveCallback_Execute(const moveit_msgs::ExecuteKnownTrajectoryGoalConstPtr& goal, moveit_msgs::ExecuteKnownTrajectoryResult &action_res)
{
  ROS_INFO("Execution request received for ExecuteKnownTrajectory action.");

  context_->trajectory_execution_manager_->clear();
  if (context_->trajectory_execution_manager_->push(goal->trajectory))
  {
    setExecuteState(MONITOR);
    context_->trajectory_execution_manager_->execute();
    moveit_controller_manager::ExecutionStatus es = context_->trajectory_execution_manager_->waitForExecution();
    if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else
    {
      if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
      {
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
      }
      else
      {
        if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
        {
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
        }
        else
        {
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
        }
      }
      ROS_INFO_STREAM("Execution completed: " << es.asString());
    }
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
}

void move_group::MoveGroupExecuteAction::preemptMoveCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

void move_group::MoveGroupExecuteAction::setExecuteState(MoveGroupState state)
{
  execute_state_ = state;
  execute_feedback_.state = stateToStr(state);
  execute_action_server_->publishFeedback(execute_feedback_);
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteAction, move_group::MoveGroupCapability)
