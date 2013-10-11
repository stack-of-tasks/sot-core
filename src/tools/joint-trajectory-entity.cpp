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
  OneStepOfUpdateS(boost::bind(&SotJointTrajectoryEntity::OneStepOfUpdate,this,_1,_2),
                   refresherSINTERN << trajectorySIN, 
                   "SotJointTrajectory("+n+")::onestepofupdate"),
  positionSOUT(boost::bind(&SotJointTrajectoryEntity::getNextPosition,
                           this,_1,_2),
               OneStepOfUpdateS,
               "SotJointTrajectory("+n+")::output(vector)::position"),
  comSOUT(boost::bind(&SotJointTrajectoryEntity::getNextCoM,
                           this,_1,_2),
               OneStepOfUpdateS,
               "SotJointTrajectory("+n+")::output(vector)::com"),
  zmpSOUT(boost::bind(&SotJointTrajectoryEntity::getNextCoP,
                           this,_1,_2),
          OneStepOfUpdateS,
          "SotJointTrajectory("+n+")::output(vector)::zmp"),
  waistSOUT(boost::bind(&SotJointTrajectoryEntity::getNextWaist,
                       this,_1,_2),
           OneStepOfUpdateS,
           "SotJointTrajectory("+n+")::output(vector)::waist"),
  trajectorySIN(NULL,"SotJointTrajectory("+n+")::input(trajectory)::trajectoryIN")
{
  signalRegistration( positionSOUT << comSOUT << zmpSOUT 
                      << waistSOUT << trajectorySIN);
  refresherSINTERN.setDependencyType( TimeDependency<int>::ALWAYS_READY );
}

int & SotJointTrajectoryEntity::OneStepOfUpdate(int &dummy,const int & time)
{
  
  const Trajectory &aTrajectory= trajectorySIN(time);
  if (traj_timestamp_==aTrajectory.header_.stamp_)
    index_++; else index_=0;

  if (index_== aTrajectory.points_.size())
    index_=aTrajectory.points_.size()-1;
    
  const JointTrajectoryPoint &aJTP=aTrajectory.points_[index_];
  std::vector<JointTrajectoryPoint>::size_type possize = 
    aJTP.positions_.size();
  pose_.resize(aJTP.positions_.size());
  for(std::vector<JointTrajectoryPoint>::size_type i=0;
      i<possize-5;i++)
    pose_(i) = aJTP.positions_[i];

  com_.resize(3);
  for(std::vector<JointTrajectoryPoint>::size_type i=possize-5,j=0;
      i<possize-2;i++,j++)
    com_(j) = aJTP.positions_[i];

  waist_.resize(3);
  for(std::vector<JointTrajectoryPoint>::size_type i=possize-5,j=0;
      i<possize-3;i++,j++)
    waist_(j) = aJTP.positions_[i];
  waist_(2) = 0.65;
  waist_(3) = com_(2);

  cop_.resize(3);
  for(std::vector<JointTrajectoryPoint>::size_type i=possize-2,j=0;
      i<possize;i++,j++)
    cop_(j) = aJTP.positions_[i];

  return dummy;
}

ml::Vector &SotJointTrajectoryEntity::
getNextPosition(ml::Vector &pos,
                const int & time)
{
  OneStepOfUpdateS(time);
  pos = pose_;
  return pos;
}
   
ml::Vector &SotJointTrajectoryEntity::
getNextCoM(ml::Vector &com,
                const int & time)
{
  OneStepOfUpdateS(time);
  com = com_;
  return com;
}

ml::Vector &SotJointTrajectoryEntity::
getNextCoP(ml::Vector &cop,
                const int & time)
{
  OneStepOfUpdateS(time);
  cop = cop_;
  return cop;
}

