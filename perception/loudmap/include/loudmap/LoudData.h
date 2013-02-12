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

#ifndef LOUDMAP_LOUDDATA_H
#define LOUDMAP_LOUDDATA_H

//TODO: isn't it good practice to avoid includes in h-files?
#include <vector>
#include <list>

#include <loudmap/loudmap_utils.h>
#include <loudmap/loudmap_types.h>
#include <loudmap/LoudListener.h>
#include <loudmap/LoudParameters.h>

namespace loudmap {

  /**
   * a convenient typedef for a list of listeners and pointers to such lists.
   */
  struct ListenerItem
  {
    tParameter data_;
    LoudListener * listener_;
  };

  typedef std::list<ListenerItem> ListenerList;
  typedef ListenerList::iterator ListenerListIterator;
  typedef std::vector<ListenerItem> ListenerVector;

  /**
   * loudmap notification data structure: stores timestamped pointers to notifiable
   * objects and a previous occupancy value to use for deciding whether notification
   * is necessary. A LoudData structure is associated to a loudmap cell.
   *
   */
    class LoudData{

    public:

    LoudData(): 
      previousOccupancy_(-1.0), 
      listeners_(NULL), 
      parameters_(NULL)
    {
    }

    LoudData(const LoudData& rhs): 
      previousOccupancy_(rhs.previousOccupancy_), 
      listeners_(NULL), 
      parameters_(NULL)
    {
      if(NULL != rhs.listeners_)
      {
        listeners_ = (new ListenerList(*(rhs.listeners_)));
        parameters_ = rhs.parameters_;
      }
    }

    ~LoudData()
    {
      if(NULL != listeners_)
      {
        listeners_->clear();
        delete listeners_;
      };
    };

    /**
     * Float value for previous occupancy value
     */
    /*TODO: float or bool?*/
      float previousOccupancy_;
    /**
     * Pointers to notifiable objects. The member is itself a pointer; only need to
     * allocate a list structure if the loudmap node actually has objects listening
     * to it.
     */
    /*TODO: list or vector? (lists are easy to merge in case of pruning)*/
    ListenerList * listeners_;

    LoudParameters const * parameters_;

    };

} // end namespace

#endif //ndef LOUDMAP_LOUDDATA_H
