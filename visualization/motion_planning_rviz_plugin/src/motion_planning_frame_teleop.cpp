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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_state/conversions.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

void MotionPlanningFrame::teleop_period_spinbox_changed(int value)
{
  ROS_INFO("Teleop period is now % 3d ms", value );
}

void MotionPlanningFrame::teleop_disable_button_clicked(void)
{
  teleop_state_ = DISABLED;
  updateTeleopModeButtons();

  //planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanButtonClicked, this));
}

void MotionPlanningFrame::teleop_jt_button_clicked(void)
{
  teleop_state_ = TELEOP_JT;
  updateTeleopModeButtons();

  //planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this));
}

void MotionPlanningFrame::teleop_ik_button_clicked(void)
{
  teleop_state_ = TELEOP_IK;
  updateTeleopModeButtons();

  //planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this));
}

void MotionPlanningFrame::teleop_mp_button_clicked(void)
{
  teleop_state_ = TELEOP_MP;
  updateTeleopModeButtons();

  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTeleopUpdate, this));
}

void MotionPlanningFrame::teleop_cvx_button_clicked(void)
{
  teleop_state_ = TELEOP_CVX;
  updateTeleopModeButtons();

  //planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this));

}

void MotionPlanningFrame::updateTeleopModeButtons(void)
{
  ui_->teleop_disable_button->setChecked(teleop_state_ == DISABLED);
  ui_->teleop_jt_button->setChecked(teleop_state_ == TELEOP_JT);
  ui_->teleop_ik_button->setChecked(teleop_state_ == TELEOP_IK);
  ui_->teleop_mp_button->setChecked(teleop_state_ == TELEOP_MP);
  ui_->teleop_cvx_button->setChecked(teleop_state_ == TELEOP_CVX);
}

void MotionPlanningFrame::computeTeleopUpdate()
{
  ROS_ERROR("v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v");
  ros::Duration target_period = ros::Duration(ui_->teleop_period_spinbox->value());
  ros::Time update_start_time = ros::Time::now();

  switch(teleop_state_)
  {
    case(TELEOP_JT):
      ROS_WARN("TELEOP_JT is not implemented!");
      break;
    case(TELEOP_IK):
      ROS_WARN("TELEOP_IK is not implemented!");
      break;
    case(TELEOP_MP):
      //ROS_WARN("TELEOP_MP is not implemented!");
      computeTeleopMPUpdate();
      break;
    case(TELEOP_CVX):
      ROS_WARN("TELEOP_CVX is not implemented!");
      break;
    case(DISABLED):
      ROS_WARN("It seems teleop was DISABLED sometime between the last queueing action and now!");
      break;
    default:
      ROS_ERROR("An unhandled teleop state was requested.");
  }

  ros::Duration time_used = ros::Time::now() - update_start_time;
  ros::Duration remaining_time = target_period - time_used;
  ROS_INFO("Time used: %.3f sec, sleeping for %.3f sec", time_used.toSec(), remaining_time.toSec());
  remaining_time.sleep();

  // When all is done, it gets ready to call itself again!
  if(teleop_state_ != DISABLED)
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeTeleopUpdate, this));
}

void MotionPlanningFrame::computeTeleopMPUpdate()
{
  if (!move_group_)
    return;

  ros::Duration target_period = ros::Duration(ui_->teleop_period_spinbox->value());
  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));

  if(current_plan_ && current_plan_->trajectory_.joint_trajectory.points.size() >= 1)
  {
    //moveit_msgs::RobotState current_plan_start_state = current_plan_->start_state_;

    size_t trajectory_index = current_plan_->trajectory_.joint_trajectory.points.size() - 1;
    for( ; trajectory_index >=0; trajectory_index--)
    {
      ROS_INFO("Trajectory index %zd of %zd. Time: %.3f vs. %.3f ",
               trajectory_index,
               current_plan_->trajectory_.joint_trajectory.points.size(),
               current_plan_->trajectory_.joint_trajectory.points[trajectory_index].time_from_start.toSec(),
               target_period.toSec());
      if( current_plan_->trajectory_.joint_trajectory.points[trajectory_index].time_from_start < target_period )
        break;
      // maybe need to bake in the delay between cycles too...
    }
    trajectory_index = std::min(trajectory_index + 1, current_plan_->trajectory_.joint_trajectory.points.size() - 1);
//    ROS_INFO("There are %zd joint names and %zd joint positions",
//             current_plan_->trajectory_.joint_trajectory.joint_names.size(),
//             current_plan_->trajectory_.joint_trajectory.points[trajectory_index].positions.size() );
    ROS_INFO("Using index %zd", trajectory_index);
    //future_start_state->setStateValues(current_plan_->trajectory_.joint_trajectory.joint_names, current_plan_->trajectory_.joint_trajectory.points[trajectory_index].positions);
  }

  //configureForPlanning();
  planning_display_->setQueryStartState(future_start_state);
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
  move_group_->setPlanningTime(target_period.toSec());
  move_group_->setWorkspace(ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0,
                            ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0,
                            ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0,
                            ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0,
                            ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0,
                            ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0);

  // Do I need to copy the plan to a new variable?
  current_plan_.reset(new move_group_interface::MoveGroup::Plan());
  if (move_group_->plan(*current_plan_))
  {
    move_group_->asyncExecute(*current_plan_);
  }
  else
    current_plan_.reset();

  //move_group_->move();
}

}
