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

    LoudTreeNode::~LoudTreeNode()
    {
      if(NULL != loudData_)
      {
        delete loudData_;
        loudData_ = NULL;
      }
      if(NULL != children)
      {
        for(int k = 0; k < 8; k++)
        {
          if(NULL != children[k])
          {
            delete static_cast<LoudTreeNode*>(children[k]);
          }
        }
        delete[] children;
        children = NULL;
      }
    }

  //TODO: createChild and deleteChild are NOT virtual in the base class. Watch out for bugs here.
  bool LoudTreeNode::createChild(unsigned int i)
  {
    if (NULL == children)
    {
      allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new LoudTreeNode();
    if(0 <= depth_)
    {
      static_cast<LoudTreeNode*>(children[i])->depth_ = depth_ - 1;
    }
    else
    {
      static_cast<LoudTreeNode*>(children[i])->depth_ = depth_;
    }
    return true;
  }

  /// Deletes the i-th child of the node
  void LoudTreeNode::deleteChild(unsigned int i)
  {
    assert((i < 8) && (NULL != children));
    assert(NULL != children[i]);
    delete static_cast<LoudTreeNode*>(children[i]);
    children[i] = NULL;
  }

  bool LoudTreeNode::registerListener(LoudListener &who, tParameter &data, LoudParameters const *parameters)
  {
    if(!who.isMoreRecent(parameters->baseTimestamp_))
    {
      return false;
    };
    if(NULL == loudData_)
    {
      loudData_ = new LoudData();
    };
    if(NULL == loudData_->listeners_)
    {
      loudData_->listeners_ = new ListenerList();
    };
    if(NULL == loudData_->parameters_)
    {
      loudData_->parameters_ = parameters;
    };
    ListenerItem newItem;
    newItem.data_ = data;
    newItem.listener_ = &who;
    loudData_->listeners_->push_back(newItem);
    loudData_->previousOccupancy_ = value;

    who.sendNotification((value < parameters->owner_->getOccupancyThresLog()), data);

    if(loudData_->parameters_->forceExpansionAtCount_ <= loudData_->listeners_->size())
    {
      expandNode();
    };
    return true;
  }

  bool LoudTreeNode::pruneNode()
  {
    //collapsible requires: all children to exist, not to have children of their own, and have the same value.
    if ((!this->collapsible()))
    {
      return false;
    }

    if(NULL != loudData_)
    {
      unsigned int listenerTotalCount = 0;
      //because of the call to collapsible above, we know that the children exist
      for(unsigned char k = 0; k < 8; k++)
      {
        if(NULL != static_cast<LoudTreeNode*>(children[k])->loudData_)
        {
          listenerTotalCount += static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_->size();
        }
      }
      if(loudData_->parameters_->preventPruningAtCount_ <= listenerTotalCount)
      {
        return false;
      }
    }

    // we know the node has children, so it shouldn't have loudData_ attached
    assert(NULL == loudData_);

    // merge the children lists into this node's loudData_
    for(unsigned char k = 0; k < 8; k++)
    {
      if(NULL != static_cast<LoudTreeNode*>(children[k])->loudData_)
      {
        if(NULL == loudData_)
        {
          loudData_ = static_cast<LoudTreeNode*>(children[k])->loudData_;
          static_cast<LoudTreeNode*>(children[k])->loudData_ = NULL;
        }
        else
        {
          if(NULL == loudData_->listeners_)
          {
            loudData_->listeners_ = static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_;
            static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_ = NULL;
          }
          else if(NULL != static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_)
          {
            loudData_->listeners_->splice(loudData_->listeners_->end(), *(static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_));
          }
        }
      }
    }

    // set value to children's values (all assumed equal)
    setValue(getChild(0)->getValue());

    // delete children
    for (unsigned char k = 0; k < 8; k++) {
      delete static_cast<LoudTreeNode*>(children[k]);
    }
    delete[] children;
    children = NULL;

    return true;
  }

  void LoudTreeNode::setLogOdds(float l)
  {
    value = l;
    sendNotifications();
  }

  void LoudTreeNode::addValue(const float& p)
  {
    value += p;
    sendNotifications();
  }

  void LoudTreeNode::expandNode()
  {
    if(0 >= depth_)
    {
      return;
    }

    assert(!hasChildren());

    for (unsigned int k = 0; k < 8; k++)
    {
      // note: this is LoudTreeNode::createChild!
      createChild(k);
      children[k]->setValue(value);
    }

    if(NULL != loudData_)
    {
      // divide the listener container among the children
      // note: does not allocate a LoudData object unless the child also has listeners attached to it
      if(NULL != loudData_->listeners_)
      {
        ListenerListIterator iterator, end;
        for(iterator = loudData_->listeners_->begin(), end = loudData_->listeners_->end(); iterator != end; iterator++)
        {
          if((*iterator).listener_->isMoreRecent(loudData_->parameters_->baseTimestamp_))
          {
            tCoordinate x, y, z;
            (*iterator).listener_->getPosition((*iterator).data_, x, y, z);

            octomap::OcTreeKey key;
            loudData_->parameters_->owner_->coordToKeyChecked(x, y, z, key);
            unsigned char k = octomap::computeChildIdx(key, depth_);

            if(NULL == static_cast<LoudTreeNode*>(children[k])->loudData_)
            {
              static_cast<LoudTreeNode*>(children[k])->loudData_ = new LoudData();
              static_cast<LoudTreeNode*>(children[k])->loudData_->previousOccupancy_ = loudData_->previousOccupancy_;
              static_cast<LoudTreeNode*>(children[k])->loudData_->parameters_ = loudData_->parameters_;
              static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_ = new ListenerList();
            }

            static_cast<LoudTreeNode*>(children[k])->loudData_->listeners_->push_back((*iterator));
          }
        }
      }

      //clear the listener container of this node
      delete loudData_;
      loudData_ = NULL;
    }
  }

  void LoudTreeNode::sendNotifications()
  {
    if((NULL == loudData_) || (NULL == loudData_->listeners_) || (NULL == loudData_->parameters_))
    {
      return;
    };
    tStatus status;
    bool oldIsFree, newIsFree;
    oldIsFree = loudData_->previousOccupancy_ < loudData_->parameters_->owner_->getOccupancyThresLog();
    newIsFree = value < loudData_->parameters_->owner_->getOccupancyThresLog();
    loudData_->previousOccupancy_ = value;
    if(oldIsFree == newIsFree)
    {
      return;
    }
    status = newIsFree;
    tTimestamp presentBase;
    presentBase.copy(loudData_->parameters_->baseTimestamp_);
    for(ListenerListIterator iterator = loudData_->listeners_->begin(), end = loudData_->listeners_->end(); iterator != end; )
    {
      if((*iterator).listener_->isMoreRecent(presentBase))
      {
        (*iterator).listener_->sendNotification(status, (*iterator).data_);
        iterator++;
      }
      else
      {
#ifdef _LOUD_USE_ONUNREGISTER_
        (*iterator).listener_->onUnregister((*iterator).data_);
#endif
        iterator = loudData_->listeners_->erase(iterator);
      }
    }
  }

  std::istream& LoudTreeNode::readValue(std::istream &s)
  {
    //identical to OcTreeDataNode::readValue, but copied here to guarantee LoudTreeNode::createChild is called
    char children_char;

    // read data:
    s.read((char*) &value, sizeof(value));
    s.read((char*)&children_char, sizeof(char));
    std::bitset<8> children ((unsigned long long) children_char);

    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        createChild(i);
        getChild(i)->readValue(s);
      }
    }
    return s;
  }


}; // end namespace

