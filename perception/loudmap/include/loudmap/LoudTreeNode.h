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

#ifndef LOUDMAP_LOUDTREENODE_H
#define LOUDMAP_LOUDTREENODE_H

//TODO: isn't it good practice to avoid includes in header files?
#include "assert.h"
#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <loudmap/loudmap_utils.h>
#include <loudmap/loudmap_types.h>
#include <loudmap/LoudData.h>
#include <loudmap/LoudListener.h>
#include <loudmap/LoudParameters.h>

namespace loudmap {

  // node definition
  class LoudTreeNode : public octomap::OcTreeNode 
  {    
    public:
    
    LoudTreeNode() : 
      octomap::OcTreeNode(), 
      loudData_(NULL),
      depth_(-1)
    {
    }

    LoudTreeNode(const LoudTreeNode& rhs) : 
      octomap::OcTreeNode(rhs), 
      loudData_(NULL),
      depth_(rhs.depth_)
    {
      if(rhs.loudData_)
      {
        loudData_ = new LoudData(*(rhs.loudData_));
      };
    }

    ~LoudTreeNode();

    // TODO: operator== is NOT virtual in the base class. This appears safe as comparison on LoudData 
    //  members is not used.
    bool operator==(const LoudTreeNode& rhs) const{
      return (rhs.value == value);
    }

    // TODO: getChild and its const version are NOT virtual in the base class. Watch out for bugs here.
    inline LoudTreeNode* getChild(unsigned int i) 
    {
      return static_cast<LoudTreeNode*> (octomap::OcTreeNode::getChild(i));
    }
    inline const LoudTreeNode* getChild(unsigned int i) const 
    {
      return static_cast<const LoudTreeNode*> (octomap::OcTreeNode::getChild(i));
    }

    //TODO: createChild and deleteChild are NOT virtual in the base class. Watch out for bugs here.
    bool createChild(unsigned int i);
    /// Deletes the i-th child of the node
    void deleteChild(unsigned int i);

    //TODO: pruneNode, expandNode, setLogOdds, addValue are NOT virtual functions in the base class. Watch out for bugs here.
    //         - make sure that all calls to pruneNode and expandNode will happen from LoudTreeNode functions
    //         (hopefully ok because LoudTree is a template type instantiated over LoudTreeNode)
    bool pruneNode();
    void expandNode();
    void setLogOdds(float l);
    //warning: this is the exact declaration from base class; do not change the const or ref parameter!
    void addValue(const float& p);


    short int getDepth(void)
    {
      return depth_;
    }
    void setDepth(short int depth)
    {
      depth_ = depth;
    }
    bool registerListener(LoudListener &who, tParameter &data, LoudParameters const * parameters);
    void sendNotifications();
    bool getPreviousOccupancy(float &previousOccupancyP) const
    {
      if(loudData_)
      {
        previousOccupancyP = loudData_->previousOccupancy_;
        return true;
      }
      return false;
    };
    bool setPreviousOccupancy(float previousOccupancyP)
    {
      if(loudData_)
      {
        loudData_->previousOccupancy_ = previousOccupancyP;
        return true;
      }
      return false;
    };
    
    // file I/O
    std::istream& readValue (std::istream &s);
    //std::ostream& writeValue(std::ostream &s) const;
    
  protected:
    LoudData *loudData_;
    short int depth_;
  };

} // end namespace

#endif //LOUDMAP_LOUDTREENODE_H
