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

#ifndef LOUDMAP_LOUDTREE_H
#define LOUDMAP_LOUDTREE_H

//TODO: isn't it good practice to avoid includes in header files?
#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <loudmap/loudmap_utils.h>
#include <loudmap/loudmap_types.h>
#include <loudmap/LoudData.h>
#include <loudmap/LoudListener.h>
#include <loudmap/LoudParameters.h>
#include <loudmap/LoudTreeNode.h>

namespace loudmap {

  class LoudTree : public octomap::OccupancyOcTreeBase<LoudTreeNode>
  {

  public:
    /// Default constructor, sets resolution of leafs
    LoudTree(double resolution) : 
      octomap::OccupancyOcTreeBase<LoudTreeNode>(resolution),
      parameters_(this) 
    {
    }

    virtual ~LoudTree()
    {
    }
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    LoudTree* create() const 
    {
      return new LoudTree(resolution);
    }

    std::string getTreeType() const 
    {
      return "LoudTree";
    }

    /// sets the threshold for occupancy (sensor model)
    void setOccupancyThres(double prob)
    {
      parameters_.threshOccupied_ = occ_prob_thres_log = octomap::logodds(prob); 
    }


    void getBaseTimestamp(tTimestamp &result) const
    {
      result.seconds_ = parameters_.baseTimestamp_.seconds_;
      result.useconds_ = parameters_.baseTimestamp_.useconds_;
    };
    void setBaseTimestamp(const tTimestamp &newTimestamp)
    {
      parameters_.baseTimestamp_.seconds_ = newTimestamp.seconds_;
      parameters_.baseTimestamp_.useconds_ = newTimestamp.useconds_;
    };

    void getOccThreshold(float &result) const
    {
      result = parameters_.threshOccupied_;
    };
    void setOccThreshold(float newThreshold)
    {
      parameters_.threshOccupied_ = newThreshold;
      occ_prob_thres_log = parameters_.threshOccupied_;
    };

    void getPruningLimit(unsigned int &result) const
    {
      result = parameters_.preventPruningAtCount_;
    };
    void setPruningLimit(unsigned int newThreshold)
    {
      parameters_.preventPruningAtCount_ = newThreshold;
    };
    void getExpansionTrigger(unsigned int &result) const
    {
      result = parameters_.forceExpansionAtCount_;
    };
    void setExpansionTrigger(unsigned int newThreshold)
    {
      parameters_.forceExpansionAtCount_ = newThreshold;
    };

    bool isNodeOccupied(const LoudTreeNode &who) const
    {
      return octomap::AbstractOccupancyOcTree::isNodeOccupied(&who);
    }

    bool registerListener(LoudListener &who, tParameter &data);

    /**
     * Manipulate log_odds value of voxel directly
     *
     * @param key OcTreeKey of the NODE that is to be updated
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual LoudTreeNode* updateNode(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval = false);

    /**
     * Manipulate log_odds value of voxel directly.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual LoudTreeNode* updateNode(const octomap::point3d& value, float log_odds_update, bool lazy_eval = false);

    /**
     * Manipulate log_odds value of voxel directly.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param x
     * @param y
     * @param z
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual LoudTreeNode* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual LoudTreeNode* updateNode(const octomap::point3d& value, bool occupied, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     *
     * @param key OcTreeKey of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual LoudTreeNode* updateNode(const octomap::OcTreeKey& key, bool occupied, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param x
     * @param y
     * @param z
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual LoudTreeNode* updateNode(double x, double y, double z, bool occupied, bool lazy_eval = false);

    /**
     * Reads only the data (=complete tree structure) from the input stream.
     * The tree needs to be constructed with the proper header information
     * beforehand, see readBinary().
     */
    std::istream& readBinaryData(std::istream &s);
    /**
     * Read all nodes from the input stream (without file header),
     * for this the tree needs to be already created.
     * For general file IO, you
     * should probably use AbstractOcTree::read() instead.
     */
    std::istream& readData(std::istream &s);


  protected:

    LoudTreeNode* searchExistingNodes(tCoordinate x, tCoordinate y, tCoordinate z);
    
    LoudParameters parameters_;

 
    //TODO: why is this useful?
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer()
         {
           LoudTree* tree = new LoudTree(0.1);
           octomap::AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer LoudTreeMemberInit_;

  };

} // end namespace

#endif //LOUDMAP_LOUDTREE_H
