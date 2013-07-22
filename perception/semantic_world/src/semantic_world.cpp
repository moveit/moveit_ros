/*********************************************************************
*
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
*********************************************************************/

/* Author: Sachin Chitta */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>

// MoveIt!
#include <moveit/semantic_world/semantic_world.h>
#include <geometric_shapes/shape_operations.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

// Eigen
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

namespace moveit
{
namespace semantic_world
{

SemanticWorld::SemanticWorld(const robot_state::Transforms& tf): tf_(tf)
{    
  table_dirty_ = false;
  table_subscriber_ = node_handle_.subscribe("table_array", 1, &SemanticWorld::tableCallback, this);
  visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualize_place", 20, true);
  collision_object_publisher_ = node_handle_.advertise<moveit_msgs::CollisionObject>("/collision_object", 20);
}

void SemanticWorld::visualizePlaceLocations(const std::vector<geometry_msgs::PoseStamped> &poses) const
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
  visualization_publisher_.publish(marker);
}

bool SemanticWorld::addTablesToCollisionWorld()
{
  boost::mutex::scoped_lock tlock(table_lock_);
  // Remove the existing tables
  for(std::size_t i=0; i < current_tables_in_collision_world_.size(); ++i)
  {    
    moveit_msgs::CollisionObject co;
    co.id = current_tables_in_collision_world_[i].first;
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);
  }
  current_tables_in_collision_world_.clear();  
  // Add the new tables
  for(std::size_t i=0; i < table_array_.tables.size(); ++i)
  {
    moveit_msgs::CollisionObject co;
    std::stringstream ss;
    ss << "table_" << i;
    co.id = ss.str();   
    current_tables_in_collision_world_.push_back(std::pair<std::string, object_recognition_msgs::Table> (co.id, table_array_.tables[i]));
    co.operation = moveit_msgs::CollisionObject::ADD;

    shapes::Shape* table_shape = shapes::constructShapeFromMsg(table_array_.tables[i].convex_hull);    
    if(!table_shape)
      continue;    

    shapes::Mesh* table_mesh = dynamic_cast<shapes::Mesh*>(table_shape);    
    shapes::Mesh* table_mesh_solid = createSolidMeshFromPlanarPolygon (*table_mesh, 0.01);
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
    
    //    if(!table_shape_msg_mesh)
    //      continue;
    
    co.meshes.push_back(table_shape_msg_mesh);
    co.mesh_poses.push_back(table_array_.tables[i].pose.pose);
    co.header = table_array_.tables[i].pose.header;    
    collision_object_publisher_.publish(co);

    delete table_shape;
    delete table_mesh_solid;      
  }  
  return true;
}

object_recognition_msgs::TableArray SemanticWorld::getTablesInROI(double minx, double miny, double minz, 
                                                                  double maxx, double maxy, double maxz)
{
  boost::mutex::scoped_lock tlock(table_lock_);
  object_recognition_msgs::TableArray tables_in_roi;
  for(std::size_t i=0; i < current_tables_in_collision_world_.size(); ++i)
  {
    if(current_tables_in_collision_world_[i].second.pose.pose.position.x >= minx &&
       current_tables_in_collision_world_[i].second.pose.pose.position.x <= maxx &&
       current_tables_in_collision_world_[i].second.pose.pose.position.y >= miny &&
       current_tables_in_collision_world_[i].second.pose.pose.position.y <= maxy &&
       current_tables_in_collision_world_[i].second.pose.pose.position.z >= minz &&
       current_tables_in_collision_world_[i].second.pose.pose.position.z <= maxz)
    {
      tables_in_roi.tables.push_back(current_tables_in_collision_world_[i].second);
    }    
  }  
  return tables_in_roi;  
}

std::vector<std::string> SemanticWorld::getTableNamesInROI(double minx, double miny, double minz, 
                                                           double maxx, double maxy, double maxz)
{
  boost::mutex::scoped_lock tlock(table_lock_);
  std::vector<std::string> result;
  for(std::size_t i=0; i < current_tables_in_collision_world_.size(); ++i)
  {
    if(current_tables_in_collision_world_[i].second.pose.pose.position.x >= minx &&
       current_tables_in_collision_world_[i].second.pose.pose.position.x <= maxx &&
       current_tables_in_collision_world_[i].second.pose.pose.position.y >= miny &&
       current_tables_in_collision_world_[i].second.pose.pose.position.y <= maxy &&
       current_tables_in_collision_world_[i].second.pose.pose.position.z >= minz &&
       current_tables_in_collision_world_[i].second.pose.pose.position.z <= maxz)
    {
      result.push_back(current_tables_in_collision_world_[i].first);
    }    
  }  
  return result;  
}

void SemanticWorld::clear()
{
  boost::mutex::scoped_lock tlock(table_lock_);
  table_array_.tables.clear(); 
  current_tables_in_collision_world_.clear();  
}

std::vector<geometry_msgs::PoseStamped> SemanticWorld::generatePlacePoses(const std::string &table_name,
                                                                          const shapes::ShapeConstPtr& object_shape,
                                                                          const geometry_msgs::Quaternion &object_orientation,
                                                                          double resolution, 
                                                                          double delta_height,
                                                                          unsigned int num_heights)
{
  boost::mutex::scoped_lock tlock(table_lock_);
  object_recognition_msgs::Table chosen_table;  

  bool found_table = false;
  for(std::size_t i=0; i < current_tables_in_collision_world_.size(); ++i)
  {
    if(current_tables_in_collision_world_[i].first == table_name)
    {
      found_table = true;
      chosen_table = current_tables_in_collision_world_[i].second;        
      break;
    }      
  }    

  std::vector<geometry_msgs::PoseStamped> place_poses;
  if(!found_table)
  {
    ROS_ERROR("Did not find table to place on");    
    return place_poses;
  }

  return generatePlacePoses(chosen_table, object_shape, object_orientation, resolution, delta_height, num_heights);
}

std::vector<geometry_msgs::PoseStamped> SemanticWorld::generatePlacePoses(const object_recognition_msgs::Table &chosen_table,
                                                                          const shapes::ShapeConstPtr& object_shape,
                                                                          const geometry_msgs::Quaternion &object_orientation,
                                                                          double resolution, 
                                                                          double delta_height,
                                                                          unsigned int num_heights) const
{
  std::vector<geometry_msgs::PoseStamped> place_poses;
  if(object_shape->type != shapes::MESH)
  {
    return place_poses;    
  }  

  double x_min(std::numeric_limits<float>::max()), x_max(-std::numeric_limits<float>::max());  
  double y_min(std::numeric_limits<float>::max()), y_max(-std::numeric_limits<float>::max());
  double z_min(std::numeric_limits<float>::max()), z_max(-std::numeric_limits<float>::max());  

  Eigen::Quaterniond rotation(object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w);  
  Eigen::Affine3d object_pose(rotation);

  const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(object_shape.get());
  
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

  double min_distance_from_edge = 0.5 * std::max<double>(fabs(x_max-x_min), fabs(y_max-y_min));
  double height_above_table = -z_min;  
  return generatePlacePoses(chosen_table, resolution, height_above_table, delta_height, num_heights, min_distance_from_edge);  
}

std::vector<geometry_msgs::PoseStamped> SemanticWorld::generatePlacePoses(const object_recognition_msgs::Table &table,
                                                                          double resolution, 
                                                                          double height_above_table,
                                                                          double delta_height,
                                                                          unsigned int num_heights,
                                                                          double min_distance_from_edge) const
{
  std::vector<geometry_msgs::PoseStamped> place_poses;
  // Assumption that the table's normal is along the Z axis
  if(table.convex_hull.vertices.empty())
     return place_poses;
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
        cv::Point2f point(point_x, point_y);
        double result = cv::pointPolygonTest(contours[0], point, true);
        if((int) result >= (int) (min_distance_from_edge*scale_factor))
        {
          Eigen::Vector3d point((double) (point_x)/scale_factor + table.x_min, 
                                (double) (point_y)/scale_factor + table.y_min, 
                                0.0);
          Eigen::Affine3d pose;
          tf::poseMsgToEigen(table.pose.pose, pose);
          point = pose * point;
          // if(place_left_ && point.y() <= 0.0)
          //  continue;
          // if(!place_left_ && point.y() > 0.0)
          //  continue;
          geometry_msgs::PoseStamped place_pose;
          place_pose.pose.orientation.w = 1.0;
          place_pose.pose.position.x = point.x();
          place_pose.pose.position.y = point.y();
          place_pose.pose.position.z = point.z() + height_above_table + mm * delta_height;
          place_pose.header = table.pose.header;
          place_poses.push_back(place_pose);
        }
      }
    }
  }
  return place_poses;
}

void SemanticWorld::tableCallback(const object_recognition_msgs::TableArrayPtr &msg)
{
  boost::mutex::scoped_lock tlock(table_lock_);
  table_array_ = *msg;
  ROS_DEBUG("Table callback with %d tables", (int) table_array_.tables.size());
  transformTableArray(table_array_);
  table_dirty_ = true;
}

void SemanticWorld::transformTableArray(object_recognition_msgs::TableArray &table_array)
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
    const Eigen::Affine3d& original_transform = tf_.getTransform(original_frame);
    Eigen::Affine3d original_pose;    
    tf::poseMsgToEigen(table_array.tables[i].pose.pose, original_pose);
    original_pose = original_transform * original_pose;
    tf::poseEigenToMsg(original_pose, table_array.tables[i].pose.pose);    
    table_array.tables[i].pose.header.frame_id = tf_.getTargetFrame();    
    ROS_DEBUG_STREAM("Successfully transformed table array from " << original_frame << 
                    "to " << table_array.tables[i].pose.header.frame_id);
    ROS_DEBUG_STREAM("Transformed pose: " << table_array.tables[i].pose.pose.position.x << "," 
                     << table_array.tables[i].pose.pose.position.y << "," 
                     << table_array.tables[i].pose.pose.position.z);
  }
}

shapes::Mesh* SemanticWorld::createSolidMeshFromPlanarPolygon (const shapes::Mesh& polygon, double thickness)
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

  shapes::Mesh* solid = new shapes::Mesh(polygon.vertex_count * 2, polygon.triangle_count * 2 + polygon.vertex_count * 2);
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

    if (triangle_normal.dot (normal) < 0)
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
}
}

}
