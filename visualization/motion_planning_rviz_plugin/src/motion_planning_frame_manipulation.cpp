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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Sachin Chitta */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

void MotionPlanningFrame::detectObjectsButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::detectObjects, this), "detect objects");
}

void MotionPlanningFrame::pickObjectButtonClicked()
{
  QList<QListWidgetItem *> sel = ui_->detected_objects_list->selectedItems();
  std::string group_name = planning_display_->getCurrentPlanningGroup();
  
  if(sel.empty())
  {
    ui_->manipulation_state->setText("No objects to pick");
    return;    
  }
  if(sel[0]->checkState() == Qt::Unchecked)
  {
    pick_object_name_[group_name] = sel[0]->text().toStdString();
  }  
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::pickObject, this), "pick");
}

void MotionPlanningFrame::placeObjectButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::placeObject, this), "place");
}

void MotionPlanningFrame::selectedDetectedObjectChanged()
{
}

void MotionPlanningFrame::detectedObjectChanged()
{
}

void MotionPlanningFrame::detectObjects()
{
  

}



}
