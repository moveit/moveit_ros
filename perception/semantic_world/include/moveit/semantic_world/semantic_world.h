/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Sachin Chitta */

#ifndef MOVEIT_SEMANTIC_WORLD_
#define MOVEIT_SEMANTIC_WORLD_

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <object_recognition_msgs/TableArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>

#include <boost/thread/mutex.hpp>

namespace moveit
{

namespace semantic_world
{

/**
 * @brief A (simple) semantic world representation for pick and place and other tasks.
 */
class SemanticWorld
{
public:

  /** @brief The signature for a callback on receiving table messages*/
  typedef boost::function<void()> SemanticWorldCallbackFn;

  /**
   * @brief A (simple) semantic world representation for pick and place and other tasks.
   * Currently this is used only to represent tables.
   */
  SemanticWorld(const planning_scene::PlanningSceneConstPtr &planning_scene);

  /**
   * @brief Get all the tables within a region of interest
   */
  object_recognition_msgs::TableArray getTablesInROI(double minx, double miny, double minz, 
                                                     double maxx, double maxy, double maxz) const;

  /**
   * @brief Get all the tables within a region of interest
   */
  std::vector<std::string> getTableNamesInROI(double minx, double miny, double minz, 
                                              double maxx, double maxy, double maxz) const;

  /**
   * @brief Generate possible place poses on the table for a given object. This chooses appropriate
   * values for min_distance_from_edge and for height_above_table based on the object properties.
   * The assumption is that the object is represented by a mesh.
   */
  std::vector<geometry_msgs::PoseStamped> generatePlacePoses(const std::string &table_name,
                                                             const shapes::ShapeConstPtr& object_shape,
                                                             const geometry_msgs::Quaternion &object_orientation,
                                                             double resolution,
                                                             double delta_height = 0.01,
                                                             unsigned int num_heights = 2) const;

  /**
   * @brief Generate possible place poses on the table for a given object. This chooses appropriate
   * values for min_distance_from_edge and for height_above_table based on the object properties.
   * The assumption is that the object is represented by a mesh.
   */
  std::vector<geometry_msgs::PoseStamped> generatePlacePoses(const object_recognition_msgs::Table &table,
                                                             const shapes::ShapeConstPtr& object_shape,
                                                             const geometry_msgs::Quaternion &object_orientation,
                                                             double resolution,
                                                             double delta_height = 0.01,
                                                             unsigned int num_heights = 2) const;
  /**
   * @brief Generate possible place poses on the table. This samples locations in a grid on the table at
   * the given resolution (in meters) in both X and Y directions. The locations are sampled at the
   * specified height above the table (in meters) and then at subsequent additional heights (num_heights
   * times) incremented by delta_height. Locations are only accepted if they are at least min_distance_from_edge
   * meters from the edge of the table.
   */
  std::vector<geometry_msgs::PoseStamped> generatePlacePoses(const object_recognition_msgs::Table &table,
                                                             double resolution,
                                                             double height_above_table,
                                                             double delta_height = 0.01,
                                                             unsigned int num_heights = 2,
                                                             double min_distance_from_edge = 0.10) const;

  /**
   * @brief Clear the set of tables in the semantic world.
   */
  void clear();

  /**
   * @brief Add the tables to the collision world by broadcasting them as a planner scene diff.
   */
  bool addTablesToCollisionWorld();

  /**
   * @brief Get a visualization marker array corresponding to possible place locations
   */
  visualization_msgs::MarkerArray getPlaceLocationsMarker(const std::vector<geometry_msgs::PoseStamped> &poses) const;

  /**
   * @brief Add a callback for tables
   */
  void addTableCallback(const SemanticWorldCallbackFn &table_callback)
  {
    table_callback_ = table_callback;
  }  

  /**
   * @brief Add a callback for recognized objects
   */
  void addRecognizedObjectCallback(const SemanticWorldCallbackFn &callback)
  {
    recognized_object_diff_callback_ = callback;
  }  

  /**
   * @brief Find the table on which this object has been placed.
   * @param pose the pose of the object (in the planning frame)
   * @param min_distance_from_edge the minimum distance that the object has to be from the 
   * edge of the table to qualify as being on the table
   * @param min_vertical_offset how far below the table the object could be to still qualify as 
   * as being on the table
   */
  std::string findObjectTable(const geometry_msgs::Pose &pose,
                              double min_distance_from_edge = 0.0,
                              double min_vertical_offset = 0.03) const;  
  
  /**
   * @brief Find whether the object is inside the table contour
   * @param pose the pose of the object (in the planning frame)
   * @param table the table that we are checking against
   * @param min_distance_from_edge the minimum distance that the object has to be from the 
   * edge of the table to qualify as being on the table
   * @param min_vertical_offset how far below the table the object could be to still qualify as 
   * as being on the table
   */
  bool isInsideTableContour(const geometry_msgs::Pose &pose,
                            const object_recognition_msgs::Table &table,
                            double min_distance_from_edge = 0.0,
                            double min_vertical_offset = 0.03) const;

  /**
   * @brief Add a collision object as a table
   * @return False if an error occurs, e.g. mesh pose is not specified
   * or mesh is not specified for the table
   */
  bool addObjectAsTable(const moveit_msgs::CollisionObject &object);  

  /**
   * @brief Add a collision object as a table
   * @param mesh The mesh for the table
   * @param pose The pose for the table
   * @param id The id for the table
   */
  bool addObjectAsTable(const shape_msgs::Mesh &mesh,
                        const geometry_msgs::Pose &pose,
                        const std::string &id);  

  /**
   * @brief Add a collision object as a table
   * @param object_shape The mesh for the table
   * @param object_pose The pose for the table
   * @param id The id for the table
   */
  bool addObjectAsTable(const shapes::ShapeConstPtr &object_shape,
                        const Eigen::Affine3d &object_pose,
                        const std::string &id);
  
  /**
   * @brief Add a table into the collision world
   * @param table The ROS msg defining a table
   * @param id The id for the table
   */
  bool addTable(const object_recognition_msgs::Table &table,
                const std::string &id);  

  /**
   * @brief Add the recognized objects into the collision world by 
   * publishing them as a planning scene diff
   */
  void publishRecognizedObjects() const;
  
  void clearAllRecognizedObjects(bool publish_planning_scene_diff);  

  void clearRecognizedObjectsInROI(double minx, double miny, double minz, 
                                   double maxx, double maxy, double maxz,
                                   bool publish_planning_scene_diff);
  
  /**
   * @brief Add a recognized collision object into the semantic world
   * @param object The recognized object to be added into the semantic world
   */
  bool addRecognizedObject(const moveit_msgs::CollisionObject &object);  

  /**
   * @brief Set the distance for the near threshold
   * @param threshold The threshold for the near distance
   */
  void setNearThreshold(double threshold);  


  std::vector<std::string> getRecognizedObjectNamesInROI(double minx, double miny, double minz, 
                                                         double maxx, double maxy, double maxz,
                                                         std::vector<std::string> &object_types) const;  

private:

  bool getTableMsgFromObject(const shapes::ShapeConstPtr &object_shape,
                             const Eigen::Affine3d &object_pose,
                             object_recognition_msgs::Table &table) const;  

  bool getTableMsgFromObject(const shape_msgs::Mesh &mesh,
                             const geometry_msgs::Pose &pose,
                             object_recognition_msgs::Table &table) const;  

  bool near(const geometry_msgs::Pose &pose_1, 
            const geometry_msgs::Pose &pose_2) const;

  shapes::Mesh* createSolidMeshFromPlanarPolygon (const shapes::Mesh& polygon, double thickness) const;

  shapes::Mesh* orientPlanarPolygon (const shapes::Mesh& polygon) const;
  
  void tableCallback(const object_recognition_msgs::TableArrayPtr &msg);  

  void recognizedObjectCallback(const moveit_msgs::PlanningScenePtr &msg);  

  void transformTableArray(object_recognition_msgs::TableArray &table_array) const;

  planning_scene::PlanningSceneConstPtr planning_scene_;

  ros::NodeHandle node_handle_;

  object_recognition_msgs::TableArray table_array_;

  std::vector<geometry_msgs::PoseStamped> place_poses_;

  std::map<std::string, object_recognition_msgs::Table> current_tables_in_collision_world_;
  
  //  boost::mutex table_lock_;

  ros::Subscriber table_subscriber_;

  ros::Subscriber recognized_object_subscriber_;

  ros::Publisher visualization_publisher_, collision_object_publisher_;  
  
  SemanticWorldCallbackFn table_callback_, recognized_object_diff_callback_;

  ros::Publisher planning_scene_diff_publisher_;  

  std::vector<moveit_msgs::CollisionObject> recognized_objects_in_collision_world_;
  
  std::map<std::string, unsigned int> recognized_object_count_;
  
  double squared_threshold_;
  
};

}

}

#endif
