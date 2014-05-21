// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008  Mikael Mayer
// Copyright  (C)  2008  Julia Jesse

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

// Modified to account for "mimic" joints, i.e. joints whose motion has a
// linear relationship to that of another joint.
// Copyright  (C)  2013  Sachin Chitta, Willow Garage

#include "moveit/lma_kinematics_plugin/chainiksolver_pos_lma_jl_mimic.hpp"
#include <ros/console.h>

namespace KDL
{

ChainIkSolverPos_LMA_JL_Mimic::ChainIkSolverPos_LMA_JL_Mimic(const Chain& _chain,
                                                           const JntArray& _q_min,
                                                           const JntArray& _q_max,
                                                           ChainFkSolverPos& _fksolver,
                                                           ChainIkSolverPos_LMA& _iksolver,
                                                           unsigned int _maxiter,
                                                           double _eps,
                                                           bool _position_ik)
  : chain(_chain),
    q_min(_q_min),
    q_min_mimic(chain.getNrOfJoints()),
    q_max(_q_max),
    q_max_mimic(chain.getNrOfJoints()),
    q_temp(chain.getNrOfJoints()),
    fksolver(_fksolver),
    iksolver(_iksolver),
    delta_q(_chain.getNrOfJoints()),
    maxiter(_maxiter),
    eps(_eps),
    position_ik(_position_ik)
{
  mimic_joints.resize(chain.getNrOfJoints());
  for(std::size_t i=0; i < mimic_joints.size(); ++i)
  { 
    mimic_joints[i].reset(i);
  }
  ROS_DEBUG_NAMED("lma","Limits");
  for(std::size_t i=0; i < q_min.rows(); ++i)
  { 
    ROS_DEBUG_NAMED("lma","%ld: Min: %f, Max: %f", long(i), q_min(i), q_max(i));
  }
  ROS_DEBUG_NAMED("lma"," ");
}

bool ChainIkSolverPos_LMA_JL_Mimic::setMimicJoints(const std::vector<lma_kinematics_plugin::JointMimic>& _mimic_joints)
{
  if(_mimic_joints.size() != chain.getNrOfJoints())
  {
    ROS_ERROR_NAMED("lma","Mimic Joint info should be same size as number of joints in chain: %d", chain.getNrOfJoints());
    return false;
  }

  for(std::size_t i=0; i < _mimic_joints.size(); ++i)
  {
    if(_mimic_joints[i].map_index >= chain.getNrOfJoints())
    {
      ROS_ERROR_NAMED("lma","Mimic Joint index should be less than number of joints in chain: %d", chain.getNrOfJoints());
      return false;
    }
  }
  mimic_joints = _mimic_joints;

  //Note that q_min and q_max will be of size chain.getNrOfJoints() - num_mimic_joints
  //  qToqMimic(q_min,q_min_mimic);
  //  qToqMimic(q_max,q_max_mimic);

  ROS_DEBUG_NAMED("lma","Set mimic joints");
  return true;
}

void ChainIkSolverPos_LMA_JL_Mimic::qToqMimic(const JntArray& q, JntArray& q_result)
{
  for(std::size_t i=0; i < chain.getNrOfJoints(); ++i)
  {
    q_result(i) = mimic_joints[i].offset + mimic_joints[i].multiplier * q(mimic_joints[i].map_index);
  }
}

void ChainIkSolverPos_LMA_JL_Mimic::qMimicToq(const JntArray& q, JntArray& q_result)
{
  for(std::size_t i=0; i < chain.getNrOfJoints(); ++i)
  {
    if(mimic_joints[i].active) //This is not a mimic joint
    {
      q_result(mimic_joints[i].map_index) = q(i);
    }
  }
}

int ChainIkSolverPos_LMA_JL_Mimic::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
{
  return CartToJntAdvanced(q_init,p_in,q_out,false);
}

void ChainIkSolverPos_LMA_JL_Mimic::harmonize(JntArray& q_out)
{
  for(size_t i=0; i< chain.getNrOfJoints(); ++i)
  {
    // Harmonize
    while(q_out(i) > 2*M_PI)
      q_out(i) -= 2*M_PI;
      
    while(q_out(i) < -2*M_PI)
      q_out(i) += 2*M_PI;
  }
}

bool ChainIkSolverPos_LMA_JL_Mimic::obeysLimits(const KDL::JntArray &q_out)
{
  bool obeys_limits = true;
  for(size_t i = 0; i < chain.getNrOfJoints(); i++)
  {
    if( (q_out(i) < (q_min(i)-0.0001)) || (q_out(i) > (q_max(i)+0.0001)) )
    {
      // One element of solution is not within limits
      obeys_limits = false;
      ROS_DEBUG_STREAM_NAMED("lma","Not in limits! " << i << " value " << q_out(i) << " has limit being  " << q_min(i) << " to " << q_max(i));
      break;
    }
  }
  return obeys_limits;
}

int ChainIkSolverPos_LMA_JL_Mimic::CartToJntAdvanced(const JntArray& q_init, const Frame& p_in, JntArray& q_out, bool lock_redundant_joints)
{
  int ik_valid = iksolver.CartToJnt(q_init, p_in, q_out);
  harmonize(q_out);
  
  if (!obeysLimits(q_out))
    ik_valid = -4;    // Doesn't obey the joint limits
    
  return ik_valid;
}

ChainIkSolverPos_LMA_JL_Mimic::~ChainIkSolverPos_LMA_JL_Mimic()
{
}

} // namespace
