/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Jon Binney, Ioan Sucan */

#ifndef MOVEIT_PERCEPTION_POINTCLOUD_OCTOMAP_UPDATER_
#define MOVEIT_PERCEPTION_POINTCLOUD_OCTOMAP_UPDATER_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace occupancy_map_monitor
{

class PointCloudOctomapUpdater : public OccupancyMapUpdater
{
public:

  PointCloudOctomapUpdater();
  virtual ~PointCloudOctomapUpdater();

  virtual bool setParams(XmlRpc::XmlRpcValue &params);

  virtual bool initialize();
  virtual void start();
  virtual void stop();
  virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr &shape);
  virtual void forgetShape(ShapeHandle handle);

protected:
  
  template<class PointT>
  void updateMask(const pcl::PointCloud<PointT> &cloud, const Eigen::Vector3d &sensor_origin, std::vector<int> &mask)
  {
  }

private:

  bool getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const;

  template<class PointT>
  void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
  {
    ROS_DEBUG("Received a new point cloud message");
    ros::WallTime start = ros::WallTime::now();

    if (monitor_->getMapFrame().empty())
      monitor_->setMapFrame(cloud_msg->header.frame_id);

    /* get transform for cloud into map frame */
    tf::StampedTransform map_H_sensor;
    if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
      map_H_sensor.setIdentity();
    else
    {
      if (tf_)
      {
        try
        {
          tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp, map_H_sensor);
        }
        catch (tf::TransformException& ex)
        {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
          return;
        }
      }
      else
        return;
    }

    /* convert cloud message to pcl cloud object */
    pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    /* compute sensor origin in map frame */
    const tf::Vector3 &sensor_origin_tf = map_H_sensor.getOrigin();
    octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
    Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

    if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp))
    {
      ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
      return;
    }

    /* mask out points on the robot */
    shape_mask_->maskContainment<PointT>(cloud, sensor_origin_eigen, 0.0, max_range_, mask_);
    updateMask<PointT>(cloud, sensor_origin_eigen, mask_);

    octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
    boost::scoped_ptr<pcl::PointCloud<PointT> > filtered_cloud;
    if (!filtered_cloud_topic_.empty())
      filtered_cloud.reset(new pcl::PointCloud<PointT>());

    tree_->lockRead();

    try
    {
      /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
       * should be occupied */
      for (unsigned int row = 0; row < cloud.height; row += point_subsample_)
      {
        unsigned int row_c = row * cloud.width;
        for (unsigned int col = 0; col < cloud.width; col += point_subsample_)
        {
          //if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
          //  continue;
          const PointT &p = cloud(col, row);
          PointT nan_point;
          nan_point.x = NAN; 
          nan_point.y = NAN; 
          nan_point.z = NAN;
          /* check for NaN */
          if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
          {
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(p.x, p.y, p.z);

            /* occupied cell at ray endpoint if ray is shorter than max range and this point
               isn't on a part of the robot*/
            if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE) 
            {
              model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
              if (filtered_cloud_keep_organized_ && filtered_cloud)
                filtered_cloud->push_back(nan_point);
            }
            else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP) 
            {
              clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
              if (filtered_cloud_keep_organized_ && filtered_cloud)
                filtered_cloud->push_back(nan_point);
            }
            else
            {
              occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
              if (filtered_cloud)
                filtered_cloud->push_back(p);
            }
          }
          else if (filtered_cloud_keep_organized_) {
            // the point is nan but keep organized
            if (filtered_cloud)
              filtered_cloud->push_back(p);
          }
        }
      }

      /* compute the free cells along each ray that ends at an occupied cell */
      for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
        if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());

      /* compute the free cells along each ray that ends at a model cell */
      for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
        if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());

      /* compute the free cells along each ray that ends at a clipped cell */
      for (octomap::KeySet::iterator it = clip_cells.begin(), end = clip_cells.end(); it != end; ++it)
        if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());
    }
    catch (...)
    {
      tree_->unlockRead();
      return;
    }

    tree_->unlockRead();

    /* cells that overlap with the model are not occupied */
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      occupied_cells.erase(*it);

    /* occupied cells are not free */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      free_cells.erase(*it);

    tree_->lockWrite();

    try
    {
      /* mark free cells only if not seen occupied in this cloud */
      for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
        tree_->updateNode(*it, false);

      /* now mark all occupied cells */
      for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
        tree_->updateNode(*it, true);

      // set the logodds to the minimum for the cells that are part of the model
      const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
      for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
        tree_->updateNode(*it, lg);
    }
    catch (...)
    {
      ROS_ERROR("Internal error while updating octree");
    }
    tree_->unlockWrite();
    ROS_DEBUG("Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
    tree_->triggerUpdateCallback();

    if (filtered_cloud)
    {
      sensor_msgs::PointCloud2 filtered_cloud_msg;
      pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
      filtered_cloud_msg.header = cloud_msg->header;
      if (filtered_cloud_keep_organized_) 
      {
        filtered_cloud_msg.width = cloud_msg->width;
        filtered_cloud_msg.height = cloud_msg->height;
        filtered_cloud_msg.is_dense = false;
      }
      filtered_cloud_publisher_.publish(filtered_cloud_msg);
    }

  }

  void stopHelper();

  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;
  boost::shared_ptr<tf::Transformer> tf_;

  /* params */
  std::string point_cloud_topic_;
  double scale_;
  double padding_;
  double max_range_;
  unsigned int point_subsample_;
  std::string filtered_cloud_topic_;
  ros::Publisher filtered_cloud_publisher_;
  bool filtered_cloud_keep_organized_;
  bool filtered_cloud_use_color_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_subscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;

  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its contsructor */
  octomap::KeyRay key_ray_;

  boost::scoped_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::vector<int> mask_;

};

}

#endif
