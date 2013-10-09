/*
 * Copyright 2013,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot/core/joint-trajectory-entity.hh>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;


SotJointTrajectoryEntity::
SotJointTrajectoryEntity(const std::string &n):
  Entity(n),
  index_(0),
  traj_timestamp_(0,0),
  refresherSINTERN("SotJointTrajectoryEntity("+n+")::intern(dummy)::refresher"),
  positionSOUT(boost::bind(&SotJointTrajectoryEntity::getNextPosition,
                           this,_1,_2),
               refresherSINTERN,
               "SotJointTrajectory("+n+")::output(vector)::position"),
  trajectorySIN(NULL,"SotJointTrajectory("+n+")::input(trajectory)::trajectoryIN")
{
  signalRegistration( positionSOUT << trajectorySIN);
  refresherSINTERN.setDependencyType( TimeDependency<int>::ALWAYS_READY );
}

ml::Vector &SotJointTrajectoryEntity::
getNextPosition(ml::Vector &pos,
                const int & time)
{
  const Trajectory &aTrajectory= trajectorySIN(time);
  if (traj_timestamp_==aTrajectory.header_.stamp_)
    index_++; else index_=0;

  if (index_== aTrajectory.points_.size())
    index_=aTrajectory.points_.size()-1;
    
  const JointTrajectoryPoint &aJTP=aTrajectory.points_[index_];
  
  pos.resize(aJTP.positions_.size());
  for(std::vector<JointTrajectoryPoint>::size_type i=0;
      i<aJTP.positions_.size();i++)
    {
      pos(i) = aJTP.positions_[i];
    }

  return pos;
}
   

