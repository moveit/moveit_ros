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

/* Author: Ioan Sucan, Jon Binney, Connor Brew */

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_
#define MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_

#include <octomap/octomap.h>
#include <boost/shared_ptr.hpp>

namespace occupancy_map_monitor
{

typedef octomap::OcTreeNode OccMapNode;

class OccMapTree : public octomap::OcTree
{
public:
  OccMapTree(double resolution) : octomap::OcTree(resolution), bbx_size(0.0), bbx_height(0.0) {}
  OccMapTree(double resolution, double size) : octomap::OcTree(resolution), bbx_size(size), bbx_height(size) {}
  OccMapTree(double resolution, double size, double height) : octomap::OcTree(resolution), bbx_size(size), bbx_height(height) {}
  OccMapTree(const std::string &filename) : octomap::OcTree(filename), bbx_size(0.0), bbx_height(0.0) {}
  OccMapTree(const OccMapTree& rhs) : octomap::OcTree(rhs), bbx_size(rhs.bbx_size), bbx_height(rhs.bbx_height) {}

  void pruneBBX()
  {
    if (!root || bbx_size <= 0.0)
      return;
    octomap::OcTreeKey root_key;
    root_key[0] = root_key[1] = root_key[2] = tree_max_val;
    if (pruneBBXRecurs(root, root_key, 0))
      tree_size = this->calcNumNodes();
  }

  void setBBXCenter(double x, double y, double z)
  {
    octomap::point3d min(x-bbx_size, y-bbx_size, z-bbx_height);
    octomap::point3d max(x+bbx_size, y+bbx_size, z+bbx_height);
    setBBXMin(min);
    setBBXMax(max);
  }

  void setBBXSize(double size)
  {
    bbx_size = size;
  }

  double getBBXSize()
  {
    return bbx_size;
  }

  void setBBXHeight(double height)
  {
    bbx_height = height;
  }

  double getBBXHeight()
  {
    return bbx_height;
  }

protected:
  bool inBBX(const octomap::OcTreeKey& key, unsigned int depth) const;
  bool inBBXStrict(const octomap::OcTreeKey& key, unsigned int depth) const;
  bool pruneBBXRecurs(octomap::OcTreeNode* node, const octomap::OcTreeKey& parent_key, unsigned int depth);

  double bbx_size;
  double bbx_height;
};

typedef boost::shared_ptr<OccMapTree> OccMapTreePtr;
typedef boost::shared_ptr<const OccMapTree> OccMapTreeConstPtr;

}

#endif
