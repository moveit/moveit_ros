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

#ifndef LOUD_LOUDLISTENER_H
#define LOUD_LOUDLISTENER_H

//TODO: isn't it good practice to avoid includes in header files?
#include <loudmap/loudmap_utils.h>
#include <loudmap/loudmap_types.h>

namespace loudmap {

  /**
   * LoudListener: a base, abstract class declaring the basic
   * notification interface. Derive notifiable objects from it.
   *
   */
  class LoudListener{

    public:
    //TODO: virtual functions add some overhead; isn't some other, elegant and fast wasy to do this?

    /**
     * Declare a virtual destructor to make sure derived objects are properly deallocated, even when
     * referred to as LoudListener*.
     */
    virtual ~LoudListener() = 0;

    /**
     * Notifies the object of an occupancy change in the cell it listens to.
     * @param status contains the new cell state (occupied or free)
     * @param data contains auxiliary data: currently, the point id for the listener object, which
     *  may contain several points; example: a roadmap state may be a listening object, it may 
     *  describe the configuration of a robot arm and therefore be interested in changes in those 
     *  cells that contain the elbow and end effector points
     */
    virtual void sendNotification(const tStatus &status, const tParameter &data) = 0;

    /**
     * Obtains the position coordinates of a point of interest for the listener.
     * @param data contains auxiliary data: currently, the point id for the listener object, which
     *  may contain several points; example: a roadmap state may be a listening object, it may 
     *  describe the configuration of a robot arm and therefore be interested in changes in those 
     *  cells that contain the elbow and end effector points
     * @param x is the x coordinate (output parameter)
     * @param y is the y coordinate (output parameter)
     * @param z is the z coordinate (output parameter)
     */
    virtual void getPosition(const tParameter &data, tCoordinate &x, tCoordinate &y, tCoordinate &z) const = 0;


    /**
     * Obtains the timestamp for the pointer to the listener object. Only if it's more recent than
     * the base timestamp will the listening object be notified of changes.
     * @param timestamp will contain the timestamp for the pointer (output parameter)
     */
    virtual void getTimestamp(tTimestamp &timestamp) const = 0;

    /**
     * Returns true if the listener's timestamp is more recent than the base.
     * @param timestamp will contain the timestamp to check against
     */
    bool isMoreRecent(const tTimestamp &timestamp) const
    {
      tTimestamp own;
      getTimestamp(own);
      return(own.isMoreRecent(timestamp));
    }

    /**
     * Notifies the listening object that the cell has unregistered its pointer, because its pointer
     * expired (its timestamp is older than the base timestamp).
     * Use of this function is optional; it's yet to be determined if it's actually necessary.
     * @param data contains auxiliary data: currently, the point id for the listener object, which
     *  may contain several points; example: a roadmap state may be a listening object, it may 
     *  describe the configuration of a robot arm and therefore be interested in changes in those 
     *  cells that contain the elbow and end effector points
     */
    virtual void onUnregister(const tParameter &data) = 0;
  };

} // end namespace

#endif //ndef LOUD_LOUDLISTENER_H
