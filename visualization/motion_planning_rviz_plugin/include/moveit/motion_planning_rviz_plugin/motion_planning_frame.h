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

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_

#include <QWidget>
#include <QTreeWidgetItem>
#include <QListWidgetItem>

#ifndef Q_MOC_RUN
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#endif

#include <moveit_msgs/MotionPlanRequest.h>
#include <map>
#include <string>

namespace rviz
{
class DisplayContext;
}

namespace Ui
{
class MotionPlanningUI;
}

namespace moveit_warehouse
{
class PlanningSceneStorage;
class ConstraintsStorage;
class RobotStateStorage;
}

namespace moveit_rviz_plugin
{
class MotionPlanningDisplay;
  
class MotionPlanningFrame : public QWidget
{
  friend class MotionPlanningDisplay;
  Q_OBJECT
  
public:
  MotionPlanningFrame(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent = 0);
  ~MotionPlanningFrame(void);

  void changePlanningGroup(void);
  void enable(void);
  void disable(void);
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq);
  
  void updateSceneMarkers(float wall_dt, float ros_dt);
  void updateGoalPoseMarkers(float wall_dt, float ros_dt);


  MotionPlanningDisplay *planning_display_;  
  rviz::DisplayContext* context_;
  Ui::MotionPlanningUI *ui_;
  
  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;
  ros::WallTime move_group_construction_time_;

  boost::shared_ptr<move_group_interface::MoveGroup::Plan> current_plan_;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;
  boost::shared_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  boost::shared_ptr<moveit_warehouse::RobotStateStorage> robot_state_storage_;

  boost::shared_ptr<rviz::InteractiveMarker> scene_marker_;
 
  class GoalPoseMarker
  {
  public:
    boost::shared_ptr<rviz::InteractiveMarker> imarker;
    bool selected;
    
    GoalPoseMarker(): selected(false) {}
    GoalPoseMarker(boost::shared_ptr<rviz::InteractiveMarker> marker): imarker(marker), selected(false) {}
    GoalPoseMarker(boost::shared_ptr<rviz::InteractiveMarker> marker, bool is_selected): imarker(marker), selected(is_selected) {}
  };
  
  typedef std::map<std::string, GoalPoseMarker> GoalPoseMap;
  typedef std::pair<std::string, GoalPoseMarker> GoalPosePair;
  GoalPoseMap goal_poses_;
          
  class StartState
  {
  public:
    moveit_msgs::RobotState state_msg;
    bool selected;
    
    StartState(): selected(false) {}
    StartState(moveit_msgs::RobotState state): state_msg(state), selected(false) {}
    StartState(moveit_msgs::RobotState state, bool is_selected): state_msg(state), selected(is_selected) {}
  };
  
  typedef std::map<std::string, StartState> StartStateMap;
  typedef std::pair<std::string, StartState> StartStatePair;
  StartStateMap start_states_;
                             
private Q_SLOTS:

  void planButtonClicked(void);  
  void executeButtonClicked(void);
  void planAndExecuteButtonClicked(void);
  void randomStatesButtonClicked(void);
  void setStartToCurrentButtonClicked(void);
  void setGoalToCurrentButtonClicked(void);
  void databaseConnectButtonClicked(void);
  void saveSceneButtonClicked(void);
  void planningSceneItemClicked(void);
  void saveQueryButtonClicked(void);
  void deleteSceneButtonClicked(void);
  void deleteQueryButtonClicked(void);
  void loadSceneButtonClicked(void);
  void loadQueryButtonClicked(void);
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void planningAlgorithmIndexChanged(int index);
  void importFileButtonClicked(void);
  void importUrlButtonClicked(void);
  void clearSceneButtonClicked(void);
  void sceneScaleChanged(int value);
  void sceneScaleStartChange(void);
  void sceneScaleEndChange(void);
  void removeObjectButtonClicked(void);
  void selectedCollisionObjectChanged(void);
  void objectPoseValueChanged(double value);
  void publishSceneButtonClicked(void);
  void collisionObjectChanged(QListWidgetItem *item);
  void pathConstraintsIndexChanged(int index);
  void imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void warehouseItemNameChanged(QTreeWidgetItem *item, int column);
  void tabChanged(int index);
  void copySelectedCollisionObject(void);
  
  //Slots for goal poses (constraints)
  void createGoalPoseButtonClicked(void);
  void removeSelectedGoalsButtonClicked(void);
  void removeAllGoalsButtonClicked(void);
  void goalPoseSelectionChanged();
  void goalPoseDoubleClicked(QListWidgetItem *item);
  void copySelectedGoalPoses(void);

  //Slots for start states
  void saveStartStateButtonClicked(void);
  void removeSelectedStatesButtonClicked(void);
  void removeAllStatesButtonClicked(void);
  void startStateItemDoubleClicked(QListWidgetItem * item);

  //Slots for goal poses and start states loading/storing/removing on the warehouse
  void loadGoalsAndStatesFromDBButtonClicked(void);
  void saveGoalsAndStatesOnDBButtonClicked(void);
  void deleteGoalsAndStatesOnDBButtonClicked(void);

private:

  void computePlanButtonClicked(void);  
  void computeExecuteButtonClicked(void);
  void computePlanAndExecuteButtonClicked(void); 
  void computePlanAndExecuteButtonClickedDisplayHelper(void);
  void computeRandomStatesButtonClicked(void);
  void computeSetStartToCurrentButtonClicked(void);
  void computeSetGoalToCurrentButtonClicked(void);
  void populatePlanningSceneTreeView(void);
  void computeDatabaseConnectButtonClicked(void);
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeSaveSceneButtonClicked(void);
  void computeSaveQueryButtonClicked(const std::string &scene, const std::string &query_name);
  void computeDeleteSceneButtonClicked(void);
  void computeDeleteQueryButtonClicked(void);
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s);
  void checkPlanningSceneTreeEnabledButtons(void);
  void computeLoadSceneButtonClicked(void);
  void computeLoadQueryButtonClicked(void);
  void populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc);
  void changePlanningGroupHelper(void);
  void populateCollisionObjectsList(void);
  void populateConstraintsList(void);
  void populateConstraintsList(const std::vector<std::string> &constr);
  void configureForPlanning(void);
  void addObject(const collision_detection::CollisionWorldPtr &world, const std::string &id,
                 const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);
  void createSceneInteractiveMarker(void);
  void renameCollisionObject(QListWidgetItem *item);
  void attachDetachCollisionObject(QListWidgetItem *item);
  void populateGoalPosesList();
  void populateStartStatesList();
  void importResource(const std::string &path);
  
  /** Selects or unselects a item in a list by the item name */
  void setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list);
  
  /** Switches the selection state of a goal pose marker */
  void switchGoalPoseMarkerSelection(const std::string &marker_name);
 
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;
  std::vector< std::pair<std::string, bool> > known_collision_objects_;
  
  EigenSTL::map_string_Affine3d goals_initial_pose_;
};

}

#endif

