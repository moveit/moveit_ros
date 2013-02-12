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

#ifndef LOUDMAP_LOUDMAP_TYPES_H
#define LOUDMAP_LOUDMAP_TYPES_H

namespace loudmap {

  typedef bool tStatus;
  typedef unsigned long int tParameter;
  typedef double tCoordinate;

  typedef struct tTimestampA
  {
    unsigned long seconds_, useconds_;
    tTimestampA(): 
      seconds_(0), 
      useconds_(0)
    {
    }

    void copy(const tTimestampA &who)
    {
      seconds_ = who.seconds_;
      useconds_ = who.useconds_;
    }

    bool isMoreRecent(const tTimestampA &who) const
    {
      return((seconds_ > who.seconds_) || (useconds_ > who.useconds_));
    }
  }tTimestamp;

}

  //Macros for compiling with and without ROS (for output logging)
  #ifdef LOUDMAP_ROS
    #include <ros/ros.h>

    #define LOUDMAP_DEBUG         ROS_DEBUG
    #define LOUDMAP_DEBUG_STR     ROS_DEBUG_STREAM
    #define LOUDMAP_WARNING       ROS_WARN
    #define LOUDMAP_WARNING_STR   ROS_WARN_STREAM
    #define LOUDMAP_ERROR         ROS_ERROR
    #define LOUDMAP_ERROR_STR     ROS_ERROR_STREAM

  #else
    // no debug output if not in debug mode:
    #ifdef NDEBUG
      #ifndef LOUDMAP_NODEBUGOUT
        #define LOUDMAP_NODEBUGOUT
      #endif
    #endif

    #ifdef LOUDMAP_NODEBUGOUT
      #define LOUDMAP_DEBUG(...)       (void)0
      #define LOUDMAP_DEBUG_STR(...)   (void)0
    #else
      #define LOUDMAP_DEBUG(...)        fprintf(stderr, __VA_ARGS__), fflush(stderr)
      #define LOUDMAP_DEBUG_STR(args)   std::cerr << args << std::endl
    #endif

    #define LOUDMAP_WARNING(...)      fprintf(stderr, "WARNING: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
    #define LOUDMAP_WARNING_STR(args) std::cerr << "WARNING: " << args << std::endl
    #define LOUDMAP_ERROR(...)        fprintf(stderr, "ERROR: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
    #define LOUDMAP_ERROR_STR(args)   std::cerr << "ERROR: " << args << std::endl
  #endif


#endif //ndef LOUDMAP_LOUDMAP_TYPES_H
