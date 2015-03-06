/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/point_containment_filter/shape_mask.h>
#include <geometric_shapes/body_operations.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>

point_containment_filter::ShapeMask::ShapeMask(const TransformCallback& transform_callback) :
  transform_callback_(transform_callback),
  next_handle_ (1),
  min_handle_ (1)
{
}

point_containment_filter::ShapeMask::~ShapeMask()
{
  freeMemory();
}

void point_containment_filter::ShapeMask::freeMemory()
{
  for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() ; ++it)
    delete it->body;
  bodies_.clear();
}

void point_containment_filter::ShapeMask::setTransformCallback(const TransformCallback& transform_callback)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  transform_callback_ = transform_callback;
}

geometry_msgs::Point ptFromEigen(const Eigen::Vector3d & p)
{
    tf::Vector3 ptTf;
    tf::vectorEigenToTF(p, ptTf);
    geometry_msgs::Point pt;
    tf::pointTFToMsg(ptTf, pt);
    return pt;
}

void point_containment_filter::ShapeMask::debug_send_shape(const bodies::Body* body, point_containment_filter::ShapeHandle sh, int id)
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("scaled_bodies", 1);
    if(pub.getNumSubscribers() == 0)
        return;

    std::string frame_id = "head_mount_kinect_rgb_optical_frame";
    const bodies::ConvexMesh* mesh_p = dynamic_cast<const bodies::ConvexMesh*>(body);
    if(!mesh_p) {
        return;
    }
    const bodies::ConvexMesh & mesh = *mesh_p;

    bodies::ConvexMesh & mesh_rw = const_cast<bodies::ConvexMesh&>(mesh);
    mesh_rw.correctVertexOrderFromPlanes();

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "scaled_bodies";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    Eigen::Affine3d tmp;
    if(!transform_callback_(sh, tmp)) {
        ROS_ERROR("%s: bad transform", __PRETTY_FUNCTION__);
        return;
    }
    tf::Pose tfPose;
    tf::poseEigenToTF(tmp, tfPose);
    tf::poseTFToMsg(tfPose, marker.pose);

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.r = 1;
    marker.color.a = 1.0; //0.8;

    const EigenSTL::vector_Vector3d & verts = mesh.getScaledVertices();
    const std::vector<unsigned int> & tris = mesh.getTriangles();
    ROS_INFO("%s: Mesh: %zu verts, %zu tris", __PRETTY_FUNCTION__, verts.size(), tris.size());
    for(unsigned int i = 0; i + 2 < tris.size(); i += 3) {
        marker.points.push_back(ptFromEigen(verts[tris[i]]));
        marker.points.push_back(ptFromEigen(verts[tris[i + 1]]));
        marker.points.push_back(ptFromEigen(verts[tris[i + 2]]));
    }

    pub.publish(marker);

#if 0
    // normals
    marker.type = visualization_msgs::Marker::LINE_LIST;    // fake arrow list
    marker.ns = "plane_normals";
    marker.color.r = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.points.clear();
    for(int i = 0; i < mesh.mesh_data_->planes_.size(); ++i) {
        Eigen::Vector3d mean = (verts[tris[i*3]] + verts[tris[i*3 + 1]] + verts[tris[i*3 + 2]])/3.0;
        const Eigen::Vector4f & plane = mesh.mesh_data_->planes_[i];
        printf("Plane: %d is %f %f %f - %f\n", i, plane.x(), plane.y(), plane.z(), plane.w());
        Eigen::Vector3d normal(plane.x(), plane.y(), plane.z());
        double d = plane.w();
        double mean_delta = normal.dot(mean) + d;
        printf("Mean detla: %f\n", mean_delta);
        Eigen::Vector3d start = mean;
        Eigen::Vector3d end = mean + (0.1) * normal;
        geometry_msgs::Point p1;
        p1.x = start.x();
        p1.y = start.y();
        p1.z = start.z();
        geometry_msgs::Point p2;
        p2.x = end.x();
        p2.y = end.y();
        p2.z = end.z();
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    usleep(50*1000);
    pub.publish(marker);
#endif

#if 0
    marker.ns = "containment";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.points.clear();
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;

    //for(double dx = -0.2; dx <= 0.5; dx += 0.01) {
    for(double dx = 0.2; dx <= 0.45; dx += 0.01) {
        //double dy = 0.1;
        for(double dy = -0.2; dy <= 0.5; dy += 0.01) {
            //double dz = 0.15;
            for(double dz = 0.1; dz <= 0.2; dz += 0.01) {
                Eigen::Vector3d pt(dx, dy, dz);
                std_msgs::ColorRGBA col;
                col.b = 0.0;
                col.a = 0.3;
                Eigen::Vector3d ptPose = tmp * pt;
                if(mesh.containsPoint(ptPose, false)) {
                //if(mesh.isPointInsidePlanes(pt)) {
                    col.r = 1.0;
                    col.g = 0.0;
                } else {
                    col.r = 0.0;
                    col.g = 1.0;
                }
                marker.colors.push_back(col);
                geometry_msgs::Point ptM;
                ptM.x = dx;
                ptM.y = dy;
                ptM.z = dz;
                marker.points.push_back(ptM);
            }
        }
    }

    usleep(50*1000);
    pub.publish(marker);
#endif
}

void make_shape(shapes::Shape* shape)
{
    if(shape->type != shapes::MESH)
        return;

    shapes::Mesh* mesh = dynamic_cast<shapes::Mesh*>(shape);
    if(mesh == NULL)
        return;

    delete [] mesh->vertices;
    mesh->vertex_count = 8;
    mesh->vertices = new double[mesh->vertex_count * 3];

    unsigned int i = 0;
    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.0;

    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.0;

    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.3;

    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.3;

    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.0;

    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.0;

    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.3;

    mesh->vertices[i++] = 0.3;
    mesh->vertices[i++] = 0.0;
    mesh->vertices[i++] = 0.3;
}

point_containment_filter::ShapeHandle point_containment_filter::ShapeMask::addShape(const shapes::ShapeConstPtr &shape, double scale, double padding)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  SeeShape ss;
  ss.body = bodies::createBodyFromShape(shape.get());
  if (ss.body)
  {
    ss.body->setScale(scale);
    ss.body->setPadding(padding);
    ss.volume = ss.body->computeVolume();
    ss.handle = next_handle_;
    std::pair<std::set<SeeShape, SortBodies>::iterator, bool> insert_op = bodies_.insert(ss);
    if (!insert_op.second)
      ROS_ERROR("Internal error in management of bodies in ShapeMask. This is a serious error.");
    used_handles_[next_handle_] = insert_op.first;
  }
  else
    return 0;

  ShapeHandle ret = next_handle_;
  const std::size_t sz = min_handle_ + bodies_.size() + 1;
  for (std::size_t i = min_handle_ ; i < sz ; ++i)
    if (used_handles_.find(i) == used_handles_.end())
    {
      next_handle_ = i;
      break;
    }
  min_handle_ = next_handle_;

  return ret;
}

void point_containment_filter::ShapeMask::removeShape(ShapeHandle handle)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  std::map<ShapeHandle, std::set<SeeShape, SortBodies>::iterator>::iterator it = used_handles_.find(handle);
  if (it != used_handles_.end())
  {
    delete it->second->body;
    bodies_.erase(it->second);
    used_handles_.erase(it);
    min_handle_ = handle;
  }
  else
    ROS_ERROR("Unable to remove shape handle %u", handle);
}

void point_containment_filter::ShapeMask::maskContainment(const pcl::PointCloud<pcl::PointXYZ>& data_in,
                                                          const Eigen::Vector3d &sensor_origin,
                                                          const double min_sensor_dist, const double max_sensor_dist,
                                                          std::vector<int> &mask)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  mask.resize(data_in.points.size());
  if (bodies_.empty())
    std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
  else
  {
    Eigen::Affine3d tmp;
    bspheres_.resize(bodies_.size());
    std::size_t j = 0;
    for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() ; ++it)
    {
      if (transform_callback_(it->handle, tmp))
      {
        it->body->setPose(tmp);
        it->body->computeBoundingSphere(bspheres_[j++]);
        debug_send_shape(it->body, it->handle, j);
      }
    }

    const unsigned int np = data_in.points.size();

    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    const double radiusSquared = bound.radius * bound.radius;

    // we now decide which points we keep
#pragma omp parallel for schedule(dynamic)
    for (int i = 0 ; i < (int)np ; ++i)
    {
      Eigen::Vector3d pt = Eigen::Vector3d(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      double d = pt.norm();
      int out = OUTSIDE;
      if (d < min_sensor_dist || d > max_sensor_dist)
        out = CLIP;
      else
        if ((bound.center - pt).squaredNorm() < radiusSquared)
          for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() && out == OUTSIDE ; ++it)
            if (it->body->containsPoint(pt))
              out = INSIDE;
      mask[i] = out;
    }
  }
}

int point_containment_filter::ShapeMask::getMaskContainment(const Eigen::Vector3d &pt) const
{
  boost::mutex::scoped_lock _(shapes_lock_);

  int out = OUTSIDE;
  for (std::set<SeeShape>::const_iterator it = bodies_.begin() ; it != bodies_.end() && out == OUTSIDE ; ++it)
    if (it->body->containsPoint(pt))
      out = INSIDE;
  return out;
}

int point_containment_filter::ShapeMask::getMaskContainment(double x, double y, double z) const
{
  return getMaskContainment(Eigen::Vector3d(x, y, z));
}
