/*
 * LoudMap - An Efficient Roadmap Projection library
 * http://<>.com/
 *
 * Copyright (c) 2013, <>, Willow Garage
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
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

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <loudmap/loudmap_utils.h>
#include <loudmap/loudmap_types.h>
#include <loudmap/LoudData.h>
#include <loudmap/LoudListener.h>
#include <loudmap/LoudParameters.h>
#include <loudmap/LoudTree.h>

namespace loudmap{

    LoudTreeNode* LoudTree::searchExistingNodes(tCoordinate x, tCoordinate y, tCoordinate z)
    {
      octomap::OcTreeKey key;
      if (!coordToKeyChecked(x, y, z, key))
      {
        LOUDMAP_ERROR_STR("Error in search: ["<< x <<" "<< y << " " << z << "] is out of OcTree bounds!");
        return NULL;
      }
      else
      {
        if (root == NULL)
        {
          this->root = new LoudTreeNode();
          //TODO: magic number; currently, max tree depth is 16 and that is what we want the root node to be initialized at
          this->root->setDepth(tree_depth - 1);
          this->tree_size++;
          return this->root;
        };

        unsigned int depth = tree_depth;

        octomap::OcTreeKey key_at_depth = key;
        key_at_depth = adjustKeyAtDepth(key, depth);

        LoudTreeNode* curNode(root);

        // follow nodes down to requested level (for diff = 0 it's the last level)
        for (int i = (tree_depth - 1); i >= 0; --i)
        {
          unsigned int pos = computeChildIdx(key_at_depth, i);
          if ((0 < i) && (curNode->childExists(pos)))
          {
            // cast needed: (nodes need to ensure it's the right pointer)
            curNode = static_cast<LoudTreeNode*>( curNode->getChild(pos) );
          }
          else
          {
            // force loop to end: either the node is a leaf or there are no better
            // resolution cells containing the desired point
            i = -1;
          };
        } // end for
        return curNode;
      };
    };

    bool LoudTree::registerListener(LoudListener &who, tParameter &data)
    {
      tCoordinate x, y, z;
      who.getPosition(data, x, y, z);
      LoudTreeNode *where = searchExistingNodes(x, y, z);
      if(NULL == where)
      {
        return false;
      }
      return where->registerListener(who, data, &parameters_);
    };

    LoudTreeNode* LoudTree::updateNode(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval)
    {
      //Note: code is almost similar to base class' updateNode, except for and initializing root depth_ property, if necessary

      // early abort (no change will happen).
      // may cause an overhead in some configuration, but more often helps
      LoudTreeNode* leaf = this->search(key);
      // no change: node already at threshold
      if (leaf
          && ((log_odds_update >= 0 && leaf->getLogOdds() >= this->clamping_thres_max)
          || ( log_odds_update <= 0 && leaf->getLogOdds() <= this->clamping_thres_min)))
      {
        return leaf;
      }

      if (this->root == NULL){
        this->root = new LoudTreeNode();
        //TODO: magic number; currently, max tree depth is 16 and that is what we want the root node to be initialized at
        this->root->setDepth(tree_depth - 1);
        this->tree_size++;
      }

      return updateNodeRecurs(this->root, false, key, 0, log_odds_update, lazy_eval);
    };

    LoudTreeNode* LoudTree::updateNode(const octomap::OcTreeKey& key, bool occupied, bool lazy_eval)
    {
      //Note: code is almost similar to base class' updateNode, except for and initializing root depth_ property, if necessary

      // early abort (no change will happen).
      // may cause an overhead in some configuration, but more often helps
      LoudTreeNode* leaf = this->search(key);
      // no change: node already at threshold
      if (leaf
          && ((occupied && leaf->getLogOdds() >= this->clamping_thres_max)
          || ( !occupied && leaf->getLogOdds() <= this->clamping_thres_min)))
      {
        return leaf;
      }

      if (this->root == NULL){
        this->root = new LoudTreeNode();
        //TODO: magic number; currently, max tree depth is 16 and that is what we want the root node to be initialized at
        this->root->setDepth(tree_depth - 1);
        this->tree_size++;
      }

      if (occupied)
        return updateNodeRecurs(this->root, false, key, 0, this->prob_hit_log,  lazy_eval);
      else
        return updateNodeRecurs(this->root, false, key, 0, this->prob_miss_log, lazy_eval);
    };

    LoudTreeNode* LoudTree::updateNode(const octomap::point3d& value, float log_odds_update, bool lazy_eval)
    {
      octomap::OcTreeKey key;
      if (!this->coordToKeyChecked(value, key))
        return NULL;
      return updateNode(key, log_odds_update, lazy_eval);
    };

    LoudTreeNode* LoudTree::updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval)
    {
      octomap::OcTreeKey key;
      if (!this->coordToKeyChecked(x, y, z, key))
        return NULL;
      return updateNode(key, log_odds_update, lazy_eval);
    };

    LoudTreeNode* LoudTree::updateNode(const octomap::point3d& value, bool occupied, bool lazy_eval)
    {
      octomap::OcTreeKey key;
      if (!this->coordToKeyChecked(value, key))
        return NULL;
      return updateNode(key, occupied, lazy_eval);
    };

    LoudTreeNode* LoudTree::updateNode(double x, double y, double z, bool occupied, bool lazy_eval)
    {
      octomap::OcTreeKey key;
      if (!this->coordToKeyChecked(x, y, z, key))
        return NULL;
      return updateNode(key, occupied, lazy_eval);
    };

    std::istream& LoudTree::readData(std::istream &s)
    {

      if (!s.good())
      {
        LOUDMAP_WARNING_STR(__FILE__ << ":" << __LINE__ << "Warning: Input filestream not \"good\"");
      }

      this->tree_size = 0;
      size_changed = true;

      // tree needs to be newly created or cleared externally
      if (root)
      {
        LOUDMAP_ERROR_STR("Trying to read into an existing tree.");
        return s;
      }

      root = new LoudTreeNode();
      root->setDepth(tree_depth - 1);
      root->readValue(s);
      tree_size = calcNumNodes();  // compute number of nodes
      return s;
    }

    std::istream& LoudTree::readBinaryData(std::istream &s)
    {
      // tree needs to be newly created or cleared externally
      if (this->root)
      {
        LOUDMAP_ERROR_STR("Trying to read into an existing tree.");
        return s;
      }

      this->root = new LoudTreeNode();
      this->root->setDepth(tree_depth - 1);
      this->readBinaryNode(s, this->root);
      this->size_changed = true;
      this->tree_size = octomap::OcTreeBaseImpl<LoudTreeNode,AbstractOccupancyOcTree>::calcNumNodes();  // compute number of nodes    
      return s;
    }


}; // end namespace

