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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>

// MoveIt!
#include <moveit/semantic_world/semantic_world.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/PlanningScene.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

// Eigen
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <boost/math/constants/constants.hpp>

namespace moveit
{
namespace semantic_world
{

SemanticWorld::SemanticWorld(const planning_scene::PlanningSceneConstPtr& planning_scene): planning_scene_(planning_scene),
                                                                                           squared_threshold_(0.0009)
{
  table_subscriber_ = node_handle_.subscribe("table_array", 1, &SemanticWorld::tableCallback, this);
  recognized_object_subscriber_ = node_handle_.subscribe("/recognized_object_diff", 1, &SemanticWorld::recognizedObjectCallback, this);  

  visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualize_place", 20, true);
  collision_object_publisher_ = node_handle_.advertise<moveit_msgs::CollisionObject>("/collision_object", 20);
  planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

visualization_msgs::MarkerArray SemanticWorld::getPlaceLocationsMarker(const std::vector<geometry_msgs::PoseStamped> &poses) const
{
  ROS_DEBUG("Visualizing: %d place poses", (int) poses.size());
  visualization_msgs::MarkerArray marker;
  for(std::size_t i=0; i < poses.size(); ++i)
  {
    visualization_msgs::Marker m;
    m.action = m.ADD;
    m.type = m.SPHERE;
    m.ns = "place_locations";
    m.id = i;
    m.pose = poses[i].pose;
    m.header = poses[i].header;

    m.scale.x = 0.02;
    m.scale.y = 0.02;
    m.scale.z = 0.02;

    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    marker.markers.push_back(m);
  }
  return marker;
}

bool SemanticWorld::getTableMsgFromObject(const shapes::ShapeConstPtr &object_shape,
                                          const Eigen::Affine3d &object_pose,
                                          object_recognition_msgs::Table &table) const
{
  if(object_shape->type != shapes::MESH)
    return false;
  const shapes::Mesh* mesh_shape = static_cast<const shapes::Mesh*>(object_shape.get());    
  shapes::ShapeMsg table_shape_msg;
  if(!shapes::constructMsgFromShape(mesh_shape, table_shape_msg))
    return false;    
  const shape_msgs::Mesh& table_shape_msg_mesh = boost::get<shape_msgs::Mesh> (table_shape_msg);    

  geometry_msgs::Pose pose;  
  tf::poseEigenToMsg(object_pose, pose);        

  return getTableMsgFromObject(table_shape_msg_mesh, pose, table);  
}

bool SemanticWorld::getTableMsgFromObject(const shape_msgs::Mesh &mesh,
                                          const geometry_msgs::Pose &pose,
                                          object_recognition_msgs::Table &table) const
{
  table.convex_hull = mesh;
  table.pose.pose = pose;
  table.pose.header.frame_id = planning_scene_->getTransforms().getTargetFrame();

  double x_min(std::numeric_limits<double>::max()), x_max(-std::numeric_limits<double>::max());
  double y_min(std::numeric_limits<double>::max()), y_max(-std::numeric_limits<double>::max());
  double z_min(std::numeric_limits<double>::max()), z_max(-std::numeric_limits<double>::max());

  for(std::size_t i = 0; i < mesh.vertices.size(); ++i)
  {
    if(mesh.vertices[i].x < x_min)
      x_min = mesh.vertices[i].x;
    if(mesh.vertices[i].y < y_min)
      y_min = mesh.vertices[i].y;
    if(mesh.vertices[i].x > x_max)
      x_max = mesh.vertices[i].x;
    if(mesh.vertices[i].y > y_max)
      y_max = mesh.vertices[i].y;
  }
  table.x_min = x_min;
  table.y_min = y_min;
  table.x_max = x_max;
  table.y_max = y_max;
  
  return true;  
}

bool SemanticWorld::addTable(const object_recognition_msgs::Table &table,
                             const std::string &id)
{
  current_tables_in_collision_world_[id] = table;  
}

bool SemanticWorld::addObjectAsTable(const shape_msgs::Mesh &mesh,
                                     const geometry_msgs::Pose &pose,
                                     const std::string &id)
{
  object_recognition_msgs::Table table;
  if(!getTableMsgFromObject(mesh, pose, table))
    return false;  
  return addTable(table, id);  
}

bool SemanticWorld::addObjectAsTable(const shapes::ShapeConstPtr &object_shape,
                                     const Eigen::Affine3d &object_pose,
                                     const std::string &id)
{
  object_recognition_msgs::Table table;
  if(!getTableMsgFromObject(object_shape, object_pose, table))
    return false;  
  return addTable(table, id);  
}

bool SemanticWorld::addObjectAsTable(const moveit_msgs::CollisionObject &object)
{
  object_recognition_msgs::Table table;
  if(object.mesh_poses.empty())
    return false;
  if(object.meshes.empty())
    return false;
  if(!getTableMsgFromObject(object.meshes[0], object.mesh_poses[0], table))
     return false;
  return addTable(table, object.id);  
}
  
bool SemanticWorld::addTablesToCollisionWorld()
{
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;

  // Remove the existing tables  
  std::map<std::string, object_recognition_msgs::Table>::iterator it;  
  for(it = current_tables_in_collision_world_.begin(); it != current_tables_in_collision_world_.end(); ++it)
  {
    moveit_msgs::CollisionObject co;
    co.id = it->first;
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    planning_scene.world.collision_objects.push_back(co);    
    //    collision_object_publisher_.publish(co);
  }
  
  planning_scene_diff_publisher_.publish(planning_scene);
  planning_scene.world.collision_objects.clear();  
  current_tables_in_collision_world_.clear();  
  // Add the new tables
  for(std::size_t i=0; i < table_array_.tables.size(); ++i)
  {
    moveit_msgs::CollisionObject co;
    std::stringstream ss;
    ss << "table_" << i;
    co.id = ss.str();
    current_tables_in_collision_world_[co.id] = table_array_.tables[i];
    co.operation = moveit_msgs::CollisionObject::ADD;

    shapes::Shape* table_shape = shapes::constructShapeFromMsg(table_array_.tables[i].convex_hull);
    if(!table_shape)
      continue;

    shapes::Mesh* table_mesh = static_cast<shapes::Mesh*>(table_shape);
    shapes::Mesh* table_mesh_solid = orientPlanarPolygon (*table_mesh);
    if(!table_mesh_solid)
    {
      delete table_shape;
      continue;
    }

    shapes::ShapeMsg table_shape_msg;
    if(!shapes::constructMsgFromShape(table_mesh_solid, table_shape_msg))
    {
      delete table_shape;
      delete table_mesh_solid;
      continue;
    }

    const shape_msgs::Mesh& table_shape_msg_mesh = boost::get<shape_msgs::Mesh> (table_shape_msg);

    co.meshes.push_back(table_shape_msg_mesh);
    co.mesh_poses.push_back(table_array_.tables[i].pose.pose);
    co.header = table_array_.tables[i].pose.header;
    planning_scene.world.collision_objects.push_back(co);    
    //    collision_object_publisher_.publish(co);
    delete table_shape;
    delete table_mesh_solid;
  }
  planning_scene_diff_publisher_.publish(planning_scene);
  return true;
}

object_recognition_msgs::TableArray SemanticWorld::getTablesInROI(double minx, double miny, double minz, 
                                                                  double maxx, double maxy, double maxz) const
{
  object_recognition_msgs::TableArray tables_in_roi;
  std::map<std::string, object_recognition_msgs::Table>::const_iterator it;
  for(it = current_tables_in_collision_world_.begin(); it != current_tables_in_collision_world_.end(); ++it)
  {
    if(it->second.pose.pose.position.x >= minx &&
       it->second.pose.pose.position.x <= maxx &&
       it->second.pose.pose.position.y >= miny &&
       it->second.pose.pose.position.y <= maxy &&
       it->second.pose.pose.position.z >= minz &&
       it->second.pose.pose.position.z <= maxz)
    {
      tables_in_roi.tables.push_back(it->second);
    }
  }
  return tables_in_roi;
}

std::vector<std::string> SemanticWorld::getTableNamesInROI(double minx, double miny, double minz, 
                                                           double maxx, double maxy, double maxz) const
{
  std::vector<std::string> result;
  std::map<std::string, object_recognition_msgs::Table>::const_iterator it;
  for(it = current_tables_in_collision_world_.begin(); it != current_tables_in_collision_world_.end(); ++it)
  {
    if(it->second.pose.pose.position.x >= minx &&
       it->second.pose.pose.position.x <= maxx &&
       it->second.pose.pose.position.y >= miny &&
       it->second.pose.pose.position.y <= maxy &&
       it->second.pose.pose.position.z >= minz &&
       it->second.pose.pose.position.z <= maxz)
    {
      result.push_back(it->first);
    }
  }
  return result;
}

void SemanticWorld::clear()
{
  table_array_.tables.clear(); 
  current_tables_in_collision_world_.clear();  
}

std::vector<geometry_msgs::PoseStamped> SemanticWorld::generatePlacePoses(const std::string &table_name,
                                                                          const shapes::ShapeConstPtr& object_shape,
                                                                          const geometry_msgs::Quaternion &object_orientation,
                                                                          double resolution,
                                                                          double delta_height,
                                                                          unsigned int num_heights,
                                                                          unsigned int num_orientations) const
{
  object_recognition_msgs::Table chosen_table;  
  std::map<std::string, object_recognition_msgs::Table>::const_iterator it = current_tables_in_collision_world_.find(table_name);

  if(it != current_tables_in_collision_world_.end())
  {
    chosen_table = it->second;
    return generatePlacePoses(chosen_table, object_shape, object_orientation, resolution, delta_height, num_heights);
  }

  std::vector<geometry_msgs::PoseStamped> place_poses;
  ROS_ERROR("Did not find table %s to place on", table_name.c_str());
  return place_poses;
}

std::vector<geometry_msgs::PoseStamped> SemanticWorld::generatePlacePoses(const object_recognition_msgs::Table &chosen_table,
                                                                          const shapes::ShapeConstPtr& object_shape,
                                                                          const geometry_msgs::Quaternion &object_orientation,
                                                                          double resolution,
                                                                          double delta_height,
                                                                          unsigned int num_heights,
                                                                          unsigned int num_orientations) const
{
  std::vector<geometry_msgs::PoseStamped> place_poses;
  if(object_shape->type != shapes::MESH && object_shape->type != shapes::SPHERE
     && object_shape->type != shapes::BOX && object_shape->type != shapes::CONE)
  {
    ROS_ERROR("Not an acceptable object shape type");    
    return place_poses;
  }

  double x_min(std::numeric_limits<double>::max()), x_max(-std::numeric_limits<double>::max());
  double y_min(std::numeric_limits<double>::max()), y_max(-std::numeric_limits<double>::max());
  double z_min(std::numeric_limits<double>::max()), z_max(-std::numeric_limits<double>::max());

  Eigen::Quaterniond rotation(object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w);
  Eigen::Affine3d object_pose(rotation);
  double min_distance_from_edge;
  double height_above_table;

  if(object_shape->type == shapes::MESH)
  {
    const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(object_shape.get());

    for(std::size_t i=0; i < mesh->vertex_count; ++i)
    {
      Eigen::Vector3d position(mesh->vertices[3*i],
                               mesh->vertices[3*i+1],
                               mesh->vertices[3*i+2]);
      position = object_pose * position;

      if(x_min > position.x())
        x_min = position.x();
      if(x_max < position.x())
        x_max = position.x();
      if(y_min > position.y())
        y_min = position.y();
      if(y_max < position.y())
        y_max = position.y();
      if(z_min > position.z())
        z_min = position.z();
      if(z_max < position.z())
        z_max = position.z();
    }
    min_distance_from_edge = 0.5 * std::max<double>(fabs(x_max-x_min), fabs(y_max-y_min));
    height_above_table = -z_min;
    ROS_DEBUG("Height above table is %f", height_above_table);    
  }
  else if(object_shape->type == shapes::BOX)//assuming box is being kept down upright
  {
    const shapes::Box* box = static_cast<const shapes::Box*>(object_shape.get());
    min_distance_from_edge = std::max<double>(fabs(box->size[0]), fabs(box->size[1]))/2.0;
    height_above_table = fabs(box->size[2])/2.0;
  }
  else if(object_shape->type == shapes::SPHERE)
  {
    const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(object_shape.get());
    min_distance_from_edge = sphere->radius;
    height_above_table = -sphere->radius;
  }
  else if(object_shape->type == shapes::CYLINDER) //assuming cylinder is being kept down upright
  {
    const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(object_shape.get());
    min_distance_from_edge = cylinder->radius;
    height_above_table = cylinder->length/2.0;
  }
  else if(object_shape->type == shapes::CONE)//assuming cone is being kept down upright
  {
    const shapes::Cone* cone = static_cast<const shapes::Cone*>(object_shape.get());
    min_distance_from_edge = cone->radius;
    height_above_table = cone->length/2.0;
  }

  return generatePlacePoses(chosen_table, resolution, height_above_table, delta_height, num_heights, min_distance_from_edge, num_orientations);
}

std::vector<geometry_msgs::PoseStamped> SemanticWorld::generatePlacePoses(const object_recognition_msgs::Table &table,
                                                                          double resolution,
                                                                          double height_above_table,
                                                                          double delta_height,
                                                                          unsigned int num_heights,
                                                                          double min_distance_from_edge,
                                                                          unsigned int num_orientations) const
{
  std::vector<geometry_msgs::PoseStamped> place_poses;
  // Assumption that the table's normal is along the Z axis
  if(table.convex_hull.vertices.empty())
  {
    ROS_ERROR("No vertices in convex hull");    
    return place_poses;
  }
  
  const int scale_factor = 100;
  std::vector<cv::Point2f> table_contour;
  for(std::size_t j=0; j < table.convex_hull.vertices.size(); ++j)
    table_contour.push_back(cv::Point((table.convex_hull.vertices[j].x-table.x_min)*scale_factor,
                                      (table.convex_hull.vertices[j].y-table.y_min)*scale_factor));

  double x_range = fabs(table.x_max-table.x_min);
  double y_range = fabs(table.y_max-table.y_min);
  int max_range = (int) x_range + 1;
  if(max_range < (int) y_range + 1)
    max_range = (int) y_range + 1;

  int image_scale = std::max<int>(max_range, 4);
  cv::Mat src = cv::Mat::zeros(image_scale*scale_factor, image_scale*scale_factor, CV_8UC1);

  for(std::size_t j = 0; j < table.convex_hull.vertices.size(); ++j )
  {
    cv::line(src, table_contour[j],  table_contour[(j+1)%table.convex_hull.vertices.size()],
             cv::Scalar( 255 ), 3, 8 );
  }

  unsigned int num_x = fabs(table.x_max-table.x_min)/resolution + 1;
  unsigned int num_y = fabs(table.y_max-table.y_min)/resolution + 1;

  ROS_DEBUG("Num points for possible place operations: %d %d", num_x, num_y);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
  for(std::size_t j=0; j < num_x; ++j)
  {
    int point_x = j * resolution * scale_factor;
    for(std::size_t k=0; k < num_y; ++k)
    {
      for(std::size_t mm=0; mm < num_heights; ++mm)
      {
        int point_y = k * resolution * scale_factor;
        cv::Point2f point2f(point_x, point_y);
        double result = cv::pointPolygonTest(contours[0], point2f, true);
        if((int) result >= (int) (min_distance_from_edge*scale_factor))
        {
          Eigen::Vector3d point((double) (point_x)/scale_factor + table.x_min,
                                (double) (point_y)/scale_factor + table.y_min,
                                height_above_table + mm * delta_height);
          Eigen::Affine3d pose;
          tf::poseMsgToEigen(table.pose.pose, pose);
          point = pose * point;

          geometry_msgs::PoseStamped place_pose;
          place_pose.pose.orientation.w = 1.0;
          place_pose.pose.position.x = point.x();
          place_pose.pose.position.y = point.y();
          place_pose.pose.position.z = point.z();
          place_pose.header = table.pose.header;

          for(std::size_t nn = 0; nn < num_orientations; ++nn)
          {
            tf::Matrix3x3 tf_orient;
            Eigen::Matrix3d e_orient;
            
            tf_orient.setRPY(0.0, 0.0, nn*2.0*boost::math::constants::pi<double>()/num_orientations);
            tf::matrixTFToEigen(tf_orient, e_orient);            
            e_orient = pose.rotation() * e_orient;            
            Eigen::Quaterniond e_quaternion(e_orient);
            tf::quaternionEigenToMsg(e_quaternion, place_pose.pose.orientation);            
            place_poses.push_back(place_pose);
          }          
        }
      }
    }
  }
  return place_poses;
}

bool SemanticWorld::isInsideTableContour(const geometry_msgs::Pose &pose,
                                         const object_recognition_msgs::Table &table,
                                         double min_distance_from_edge,
                                         double min_vertical_offset) const
{
  // Assumption that the table's normal is along the Z axis
  if(table.convex_hull.vertices.empty())
     return false;
  const int scale_factor = 100;
  std::vector<cv::Point2f> table_contour;
  for(std::size_t j=0; j < table.convex_hull.vertices.size(); ++j)
    table_contour.push_back(cv::Point((table.convex_hull.vertices[j].x-table.x_min)*scale_factor,
                                      (table.convex_hull.vertices[j].y-table.y_min)*scale_factor));

  double x_range = fabs(table.x_max-table.x_min);
  double y_range = fabs(table.y_max-table.y_min);
  int max_range = (int) x_range + 1;
  if(max_range < (int) y_range + 1)
    max_range = (int) y_range + 1;

  int image_scale = std::max<int>(max_range, 4);
  cv::Mat src = cv::Mat::zeros(image_scale*scale_factor, image_scale*scale_factor, CV_8UC1);

  for(std::size_t j = 0; j < table.convex_hull.vertices.size(); ++j )
  {
    cv::line(src, table_contour[j],  table_contour[(j+1)%table.convex_hull.vertices.size()],
             cv::Scalar( 255 ), 3, 8 );
  }

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  Eigen::Vector3d point(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Affine3d pose_table;
  tf::poseMsgToEigen(table.pose.pose, pose_table);

  // Point in table frame
  point = pose_table.inverse() * point;
  //Assuming Z axis points upwards for the table
  if(point.z() < -fabs(min_vertical_offset))
  {
    ROS_ERROR("Object is not above table");    
    return false;  
  }  

  int point_x = (point.x() -table.x_min) * scale_factor;
  int point_y = (point.y() -table.y_min) * scale_factor;
  cv::Point2f point2f(point_x, point_y);
  double result = cv::pointPolygonTest(contours[0], point2f, true);
  ROS_DEBUG("table distance: %f", result);
  
  if((int) result >= (int) (min_distance_from_edge*scale_factor))
    return true;

  return false;  
}

std::string SemanticWorld::findObjectTable(const geometry_msgs::Pose &pose,
                                           double min_distance_from_edge,
                                           double min_vertical_offset) const
{
  std::map<std::string, object_recognition_msgs::Table>::const_iterator it;
  for(it = current_tables_in_collision_world_.begin(); it != current_tables_in_collision_world_.end(); ++it)
  {
    ROS_DEBUG("Testing table: %s", it->first.c_str());    
    if(isInsideTableContour(pose, it->second, min_distance_from_edge, min_vertical_offset))
      return it->first;    
  }  
  return std::string();  
}


void SemanticWorld::tableCallback(const object_recognition_msgs::TableArrayPtr &msg)
{
  table_array_ = *msg;
  ROS_DEBUG("Table callback with %d tables", (int) table_array_.tables.size());
  transformTableArray(table_array_);
  // Callback on an update
  if(table_callback_)
  {
    ROS_DEBUG("Calling table callback");    
    table_callback_();  
  }  
}

void SemanticWorld::recognizedObjectCallback(const moveit_msgs::PlanningScenePtr &msg)
{
  moveit_msgs::PlanningScene diff = *msg;
  for(std::size_t i=0; i < diff.world.collision_objects.size(); ++i)
    addRecognizedObject(diff.world.collision_objects[i]);  
  // Callback on an update
  if(recognized_object_diff_callback_)
  {
    ROS_DEBUG("Calling recognized object diff callback");    
    recognized_object_diff_callback_();  
  }  
}

void SemanticWorld::transformTableArray(object_recognition_msgs::TableArray &table_array) const
{
  for(std::size_t i=0; i < table_array.tables.size(); ++i)
  {
    std::string original_frame = table_array.tables[i].pose.header.frame_id;
    if(table_array.tables[i].convex_hull.vertices.empty())
      continue;
    ROS_DEBUG_STREAM("Original pose: " << table_array.tables[i].pose.pose.position.x << "," 
                    << table_array.tables[i].pose.pose.position.y << "," 
                    << table_array.tables[i].pose.pose.position.z);
    std::string error_text;
    const Eigen::Affine3d& original_transform = planning_scene_->getTransforms().getTransform(original_frame);
    Eigen::Affine3d original_pose;
    tf::poseMsgToEigen(table_array.tables[i].pose.pose, original_pose);
    original_pose = original_transform * original_pose;
    tf::poseEigenToMsg(original_pose, table_array.tables[i].pose.pose);    
    table_array.tables[i].pose.header.frame_id = planning_scene_->getTransforms().getTargetFrame();    
    ROS_DEBUG_STREAM("Successfully transformed table array from " << original_frame << 
                    "to " << table_array.tables[i].pose.header.frame_id);
    ROS_DEBUG_STREAM("Transformed pose: " << table_array.tables[i].pose.pose.position.x << "," 
                     << table_array.tables[i].pose.pose.position.y << "," 
                     << table_array.tables[i].pose.pose.position.z);
  }
}

shapes::Mesh* SemanticWorld::orientPlanarPolygon (const shapes::Mesh& polygon) const
{
  if (polygon.vertex_count < 3 || polygon.triangle_count < 1)
   return 0;
  // first get the normal of the first triangle of the input polygon
  Eigen::Vector3d vec1, vec2, vec3, normal;

  int vIdx1 = polygon.triangles [0];
  int vIdx2 = polygon.triangles [1];
  int vIdx3 = polygon.triangles [2];
  vec1 = Eigen::Vector3d (polygon.vertices [vIdx1 * 3], polygon.vertices [vIdx1 * 3 + 1], polygon.vertices [vIdx1* 3 + 2]);
  vec2 = Eigen::Vector3d (polygon.vertices [vIdx2 * 3], polygon.vertices [vIdx2 * 3 + 1], polygon.vertices [vIdx2* 3 + 2]);
  vec3 = Eigen::Vector3d (polygon.vertices [vIdx3 * 3], polygon.vertices [vIdx3 * 3 + 1], polygon.vertices [vIdx3* 3 + 2]);
  vec2 -= vec1;
  vec3 -= vec1;
  normal = vec3.cross(vec2);

  if (normal [2] < 0.0)
    normal *= -1.0;

  normal.normalize();

  shapes::Mesh* solid = new shapes::Mesh(polygon.vertex_count, polygon.triangle_count);// + polygon.vertex_count * 2);
  solid->type = shapes::MESH;

  // copy the first set of vertices
  memcpy (solid->vertices, polygon.vertices, polygon.vertex_count * 3 * sizeof(double));
  // copy the first set of triangles
  memcpy (solid->triangles, polygon.triangles, polygon.triangle_count * 3 * sizeof(unsigned int));

  for (unsigned tIdx = 0; tIdx < polygon.triangle_count; ++tIdx)
  {
    int vIdx1 = polygon.triangles [tIdx*3];
    int vIdx2 = polygon.triangles [tIdx*3+1];
    int vIdx3 = polygon.triangles [tIdx*3+2];

    vec1 = Eigen::Vector3d (polygon.vertices [vIdx1 * 3], polygon.vertices [vIdx1 * 3 + 1], polygon.vertices [vIdx1* 3 + 2]);
    vec2 = Eigen::Vector3d (polygon.vertices [vIdx2 * 3], polygon.vertices [vIdx2 * 3 + 1], polygon.vertices [vIdx2* 3 + 2]);
    vec3 = Eigen::Vector3d (polygon.vertices [vIdx3 * 3], polygon.vertices [vIdx3 * 3 + 1], polygon.vertices [vIdx3* 3 + 2]);

    vec2 -= vec1;
    vec3 -= vec1;

    Eigen::Vector3d triangle_normal = vec2.cross(vec1);

    if (triangle_normal.dot (normal) < 0.0)
      std::swap (solid->triangles [tIdx*3 + 1], solid->triangles [tIdx*3 + 2]);

  }
  return solid;
}

shapes::Mesh* SemanticWorld::createSolidMeshFromPlanarPolygon (const shapes::Mesh& polygon, double thickness) const
{
  if (polygon.vertex_count < 3 || polygon.triangle_count < 1 || thickness <= 0)
   return 0;
  // first get the normal of the first triangle of the input polygon
  Eigen::Vector3d vec1, vec2, vec3, normal;

  int vIdx1 = polygon.triangles [0];
  int vIdx2 = polygon.triangles [1];
  int vIdx3 = polygon.triangles [2];
  vec1 = Eigen::Vector3d (polygon.vertices [vIdx1 * 3], polygon.vertices [vIdx1 * 3 + 1], polygon.vertices [vIdx1* 3 + 2]);
  vec2 = Eigen::Vector3d (polygon.vertices [vIdx2 * 3], polygon.vertices [vIdx2 * 3 + 1], polygon.vertices [vIdx2* 3 + 2]);
  vec3 = Eigen::Vector3d (polygon.vertices [vIdx3 * 3], polygon.vertices [vIdx3 * 3 + 1], polygon.vertices [vIdx3* 3 + 2]);
  vec2 -= vec1;
  vec3 -= vec1;
  normal = vec3.cross(vec2);

  if (normal [2] < 0.0)
    normal *= -1.0;

  normal.normalize();

  //shapes::Mesh* solid = new shapes::Mesh(polygon.vertex_count, polygon.triangle_count);// + polygon.vertex_count * 2);

  shapes::Mesh* solid = new shapes::Mesh(polygon.vertex_count * 2, polygon.triangle_count * 2);// + polygon.vertex_count * 2);
  solid->type = shapes::MESH;

  // copy the first set of vertices
  memcpy (solid->vertices, polygon.vertices, polygon.vertex_count * 3 * sizeof(double));
  // copy the first set of triangles
  memcpy (solid->triangles, polygon.triangles, polygon.triangle_count * 3 * sizeof(unsigned int));

  for (unsigned tIdx = 0; tIdx < polygon.triangle_count; ++tIdx)
  {
    solid->triangles [(tIdx + polygon.triangle_count) * 3 + 0] = solid->triangles [tIdx * 3 + 0] + polygon.vertex_count;
    solid->triangles [(tIdx + polygon.triangle_count) * 3 + 1] = solid->triangles [tIdx * 3 + 1] + polygon.vertex_count;
    solid->triangles [(tIdx + polygon.triangle_count) * 3 + 2] = solid->triangles [tIdx * 3 + 2] + polygon.vertex_count;

    int vIdx1 = polygon.triangles [tIdx*3];
    int vIdx2 = polygon.triangles [tIdx*3+1];
    int vIdx3 = polygon.triangles [tIdx*3+2];

    vec1 = Eigen::Vector3d (polygon.vertices [vIdx1 * 3], polygon.vertices [vIdx1 * 3 + 1], polygon.vertices [vIdx1* 3 + 2]);
    vec2 = Eigen::Vector3d (polygon.vertices [vIdx2 * 3], polygon.vertices [vIdx2 * 3 + 1], polygon.vertices [vIdx2* 3 + 2]);
    vec3 = Eigen::Vector3d (polygon.vertices [vIdx3 * 3], polygon.vertices [vIdx3 * 3 + 1], polygon.vertices [vIdx3* 3 + 2]);

    vec2 -= vec1;
    vec3 -= vec1;

    Eigen::Vector3d triangle_normal = vec2.cross(vec1);

    if (triangle_normal.dot (normal) < 0.0)
      std::swap (solid->triangles [tIdx*3 + 1], solid->triangles [tIdx*3 + 2]);
    else
      std::swap (solid->triangles [(tIdx + polygon.triangle_count) * 3 + 1], solid->triangles [(tIdx + polygon.triangle_count) * 3 + 2]);
  }

  for (unsigned vIdx = 0; vIdx < polygon.vertex_count; ++vIdx)
  {
    solid->vertices [(vIdx + polygon.vertex_count) * 3 + 0] = solid->vertices [vIdx * 3 + 0] - thickness * normal [0];
    solid->vertices [(vIdx + polygon.vertex_count) * 3 + 1] = solid->vertices [vIdx * 3 + 1] - thickness * normal [1];
    solid->vertices [(vIdx + polygon.vertex_count) * 3 + 2] = solid->vertices [vIdx * 3 + 2] - thickness * normal [2];
  }

  return solid;
}

void SemanticWorld::publishRecognizedObjects() const
{
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  for(std::size_t i=0; i < recognized_objects_in_collision_world_.size(); ++i)
  {
    planning_scene.world.collision_objects.push_back(recognized_objects_in_collision_world_[i]);    
  }
  planning_scene_diff_publisher_.publish(planning_scene);
}

void SemanticWorld::clearAllRecognizedObjects(bool publish_planning_scene_diff)
{
  recognized_object_count_.clear();
  if(publish_planning_scene_diff)
  {
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    for(std::size_t i=0; i < recognized_objects_in_collision_world_.size(); ++i)
    {
      moveit_msgs::CollisionObject co;
      co.id = recognized_objects_in_collision_world_[i].id;
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      planning_scene.world.collision_objects.push_back(co);    
    }
    planning_scene_diff_publisher_.publish(planning_scene);
  }  
  recognized_objects_in_collision_world_.clear();  
}

std::vector<std::string> SemanticWorld::getRecognizedObjectNamesInROI(double minx, double miny, double minz, 
                                                                      double maxx, double maxy, double maxz,
                                                                      std::vector<std::string> &object_types) const
{
  std::vector<std::string> object_ids;  
  std::vector<moveit_msgs::CollisionObject>::const_iterator it = recognized_objects_in_collision_world_.begin();
  for( ; it != recognized_objects_in_collision_world_.end(); ++it)
  {
    const geometry_msgs::Pose &object_pose = it->mesh_poses.empty() ? it->primitive_poses[0] : it->mesh_poses[0];

    if(object_pose.position.x >= minx &&
       object_pose.position.x <= maxx &&
       object_pose.position.y >= miny &&
       object_pose.position.y <= maxy &&
       object_pose.position.z >= minz &&
       object_pose.position.z <= maxz)
    {
      object_ids.push_back(it->id);
      object_types.push_back(it->type.key);      
    }
  }  
  return object_ids;      
}

void SemanticWorld::clearRecognizedObjectsInROI(double minx, double miny, double minz, 
                                                double maxx, double maxy, double maxz,
                                                bool publish_planning_scene_diff)
{
  std::vector<moveit_msgs::CollisionObject>::iterator it = recognized_objects_in_collision_world_.begin();
  std::vector<std::string> objects_to_remove;  
  for( ; it != recognized_objects_in_collision_world_.end(); )
  {
    const geometry_msgs::Pose &object_pose = it->mesh_poses.empty() ? it->primitive_poses[0] : it->mesh_poses[0];

    if(object_pose.position.x >= minx &&
       object_pose.position.x <= maxx &&
       object_pose.position.y >= miny &&
       object_pose.position.y <= maxy &&
       object_pose.position.z >= minz &&
       object_pose.position.z <= maxz)
    {
      if(publish_planning_scene_diff)
        objects_to_remove.push_back(it->id);      
      it = recognized_objects_in_collision_world_.erase(it);
    }
    else
    {      
      ++it;    
    }    
  }

  if(publish_planning_scene_diff)
  {
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    for(std::size_t i = 0; i < objects_to_remove.size(); ++i)
    {
      moveit_msgs::CollisionObject co;
      co.id = objects_to_remove[i];
      co.operation = moveit_msgs::CollisionObject::REMOVE;
      planning_scene.world.collision_objects.push_back(co);    
    }
    planning_scene_diff_publisher_.publish(planning_scene);
  }  
}

bool SemanticWorld::addRecognizedObject(const moveit_msgs::CollisionObject &object)
{
  if(object.type.key.empty())
  {
    ROS_ERROR("Not a recognized object, will not process");
    return false;
  }
  if(object.mesh_poses.empty() && object.primitive_poses.empty())
  {
    ROS_ERROR("No poses specified for object");
    return false;
  }
  const geometry_msgs::Pose &object_pose = object.mesh_poses.empty() ? object.primitive_poses[0] : object.mesh_poses[0];
  
  std::vector<moveit_msgs::CollisionObject>::const_iterator it;
  for(it = recognized_objects_in_collision_world_.begin(); it != recognized_objects_in_collision_world_.end(); ++it)
  {
    if(it->type.key == object.type.key)
    {
      // Check if they are really close
      const geometry_msgs::Pose &pose = it->mesh_poses.empty() ? it->primitive_poses[0]: it->mesh_poses[0];      
      if(near(object_pose, pose))
      {
        ROS_DEBUG("New object is really close to object: %s. Will not add", it->id.c_str());
        return true;        
      }
    }
  }
  // Add the new object in
  if(recognized_object_count_.find(object.type.key) == recognized_object_count_.end())
    recognized_object_count_[object.type.key] = 0;
    
  std::stringstream ss;
  ss << object.type.key << "_" << recognized_object_count_[object.type.key];
  recognized_object_count_[object.type.key]++;
  
  moveit_msgs::CollisionObject new_object = object;  
  new_object.operation = moveit_msgs::CollisionObject::ADD;
  new_object.id = ss.str();  
  recognized_objects_in_collision_world_.push_back(new_object);

  return true;
}

bool SemanticWorld::near(const geometry_msgs::Pose &pose_1, 
                         const geometry_msgs::Pose &pose_2) const
{
  double dx = pose_1.position.x - pose_2.position.x;
  double dy = pose_1.position.y - pose_2.position.y;
  double dz = pose_1.position.z - pose_2.position.z;
  
  if((dx*dx+dy*dy+dz*dz) < squared_threshold_)
    return true;
  
  return false;  
}

void SemanticWorld::setNearThreshold(double threshold)
{
  squared_threshold_ = threshold*threshold;
}

}

}
