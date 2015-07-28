/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fetch Robotics
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

/* Author: Connor Brew */

#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <ros/console.h>

namespace occupancy_map_monitor
{

bool OccMapTree::inBBX(const octomap::OcTreeKey& key, unsigned int depth) const {
  octomap::OcTreeKey bbx_min_at_depth = adjustKeyAtDepth(bbx_min_key, depth);
  octomap::OcTreeKey bbx_max_at_depth = adjustKeyAtDepth(bbx_max_key, depth);
  return ((key[0] >= bbx_min_at_depth[0]) && (key[1] >= bbx_min_at_depth[1]) && (key[2] >= bbx_min_at_depth[2]) &&
          (key[0] <= bbx_max_at_depth[0]) && (key[1] <= bbx_max_at_depth[1]) && (key[2] <= bbx_max_at_depth[2]) );
}

bool OccMapTree::inBBXStrict(const octomap::OcTreeKey& key, unsigned int depth) const {
  octomap::OcTreeKey bbx_min_at_depth = adjustKeyAtDepth(bbx_min_key, depth);
  octomap::OcTreeKey bbx_max_at_depth = adjustKeyAtDepth(bbx_max_key, depth);
  return ((key[0] > bbx_min_at_depth[0]) && (key[1] > bbx_min_at_depth[1]) && (key[2] > bbx_min_at_depth[2]) &&
          (key[0] < bbx_max_at_depth[0]) && (key[1] < bbx_max_at_depth[1]) && (key[2] < bbx_max_at_depth[2]) );
}

bool OccMapTree::pruneBBXRecurs(octomap::OcTreeNode* node, const octomap::OcTreeKey& parent_key, unsigned int depth)
{
	if (!node->hasChildren())
		return false;

  bool pruned = false;
  unsigned int child_depth = depth + 1;
  unsigned short int center_offset_key = tree_max_val >> child_depth;
  octomap::OcTreeKey child_key;

  for (int i=0; i<8; ++i)
  {
    if (node->childExists(i))
    {
      octomap::computeChildKey(i, center_offset_key, parent_key, child_key);
      if (inBBX(child_key, child_depth))
      {
        if (!inBBXStrict(child_key, child_depth))
        {
          if (pruneBBXRecurs(node->getChild(i), child_key, child_depth))
          {
          	pruned = true;
          	if (!node->getChild(i)->hasChildren()) // All of the childs children were pruned
          	{
          		node->deleteChild(i);
          	}
          }
        }
      }
      else
      {
        node->deleteChild(i);
        size_changed = true;
        pruned = true;
      }
    }
  }
  if (pruned && node->hasChildren()) {
    node->updateOccupancyChildren();
  }
  return pruned;
}

}