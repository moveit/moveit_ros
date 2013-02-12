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

#ifndef LOUDMAP_LOUDPARAMETERS_H
#define LOUDMAP_LOUDPARAMETERS_H

//TODO: isn't it good practice to avoid includes in h-files?

#include <loudmap/loudmap_utils.h>
#include <loudmap/loudmap_types.h>

namespace loudmap {

  //A few needful forward class declarations;
  class LoudTreeNode;
  class LoudTree;

  /**
   * loudmap parameter structure: stores the base timestamp for the map
   * as well as the various thresholds associated with octomap operations.
   * Also contains thresholds relating to how many listeners a cell can
   * contain before an expansion is automatically attempted, and how many
   * listeners a cell (or rather, its children) can contain before it is 
   * prevented from pruning.
   */
    class LoudParameters
    {

      public:

      LoudParameters(): 
        baseTimestamp_(), 
        threshOccupied_(0.5), 
        forceExpansionAtCount_(0xFFFFFFFF), 
        preventPruningAtCount_(0xFFFFFFFF),
        treeDepth_(16),
        owner_(NULL)
      {
      }

      LoudParameters(LoudTree const *owner): 
        baseTimestamp_(), 
        threshOccupied_(0.5), 
        forceExpansionAtCount_(0xFFFFFFFF), 
        preventPruningAtCount_(0xFFFFFFFF),
        treeDepth_(16),
        owner_(owner)
      {
      }

      /**
       * Timestamp for the base. Example, the timestamp for the latest position
       * estimate of the robot base.
       */
      tTimestamp baseTimestamp_;

    
      /**
       * Occupancy values above this will indicate the cell is occupied.
       */
      float threshOccupied_;

      /**
       * If the number of listeners to the cell reaches or exceeds this number,
       * the cell will be split.
       */
      unsigned int forceExpansionAtCount_;

      /**
       * If the number of listeners to the cell reaches or exceeds this number,
       * the cell will not be pruned.
       */
      unsigned int preventPruningAtCount_;

      /**
       * Depth of the tree (root is considered the deepest node, while max resolution leaves have depth 0).
       */
      short int treeDepth_;

      /**
       * Owner tree of this parameters structure.
       */
      LoudTree const *owner_;
    };

} // end namespace

#endif //ndef LOUDMAP_LOUDPARAMETERS_H
