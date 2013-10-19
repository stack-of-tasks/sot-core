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
//#define VP_DEBUG
#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>
#ifdef VP_DEBUG
 class sotJTE__INIT
 {
 public:sotJTE__INIT( void ) { dynamicgraph::sot::DebugTrace::openFile(); }
 };
 sotJTE__INIT sotJTE_initiator;
#endif //#ifdef VP_DEBUG


#include <jrl/mathtools/angle.hh>

#include <dynamic-graph/factory.h>

#include <sot/core/joint-trajectory-entity.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SotJointTrajectoryEntity,"SotJointTrajectoryEntity");

SotJointTrajectoryEntity::
SotJointTrajectoryEntity(const std::string &n):
  Entity(n),
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
           "SotJointTrajectory("+n+")::output(MatrixHomogeneous)::waist"),
  trajectorySIN(NULL,"SotJointTrajectory("+n+")::input(trajectory)::trajectoryIN"),
  index_(0),
  traj_timestamp_(0,0)
{
  sotDEBUGIN(5);
  signalRegistration( positionSOUT << comSOUT << zmpSOUT 
                      << waistSOUT << trajectorySIN);
  refresherSINTERN.setDependencyType( TimeDependency<int>::ALWAYS_READY );
  sotDEBUGOUT(5);
}

int & SotJointTrajectoryEntity::OneStepOfUpdate(int &dummy,const int & time)
{
  sotDEBUGIN(5);
  const Trajectory &aTrajectory= trajectorySIN(time);
  if (traj_timestamp_==aTrajectory.header_.stamp_)
    index_++; 
  else 
    {
      index_=0;
      traj_timestamp_ = aTrajectory.header_.stamp_;
    }

  sotDEBUG(5) << "step 1 " << std::endl
              << "header: " << std::endl
              << "  timestamp:" 
              << aTrajectory.header_.stamp_.secs_ +
    0.000000001 *aTrajectory.header_.stamp_.nsecs_ 
              << " seq:" << aTrajectory.header_.seq_ << " " 
              << " frame_id:" << aTrajectory.header_.frame_id_
              << std::endl;
  
  if (index_== aTrajectory.points_.size())
    {
      
      if (aTrajectory.points_.size()==0)
        {
          sotDEBUG(5) << "aTrajectory.points_.size()="
                      << aTrajectory.points_.size() << std::endl;
          return dummy;
        }

      index_=aTrajectory.points_.size()-1;
      sotDEBUG(5) << "index_=aTrajectory.points_.size()-1;" << std::endl;
    }
  
  sotDEBUG(5) << "index_:" << index_ 
              << " aTrajectory.points_.size():" << aTrajectory.points_.size()
              << std::endl;
  // Posture
  const JointTrajectoryPoint &aJTP=aTrajectory.points_[index_];
  std::vector<JointTrajectoryPoint>::size_type possize = 
    aJTP.positions_.size();
  pose_.resize(aJTP.positions_.size());
  for(std::vector<JointTrajectoryPoint>::size_type i=0;
      i<possize-5;i++)
    pose_(i) = aJTP.positions_[i];

  sotDEBUG(5) << std::endl;
  // Center of Mass
  com_.resize(3);
  for(std::vector<JointTrajectoryPoint>::size_type i=possize-5,j=0;
      i<possize-2;i++,j++)
    com_(j) = aJTP.positions_[i];

  sotDEBUG(5) << std::endl;
  // The waist is provided in the 2D plane
  ml::Vector waistXYZTheta;
  waistXYZTheta.resize(4);
  for(std::vector<JointTrajectoryPoint>::size_type i=possize-5,j=0;
      i<possize-3;i++,j++)
    waistXYZTheta(j) = aJTP.positions_[i];

  sotDEBUG(5) << std::endl;
  // Add a constant height TODO: make it variable
  waistXYZTheta(2) = 0.65; 
  waistXYZTheta(3) = com_(2);
  waist_ = XYZThetaToMatrixHomogeneous(waistXYZTheta);

  sotDEBUG(5) << std::endl;
  // Center of Pressure
  cop_.resize(3);
  for(std::vector<JointTrajectoryPoint>::size_type i=possize-2,j=0;
      i<possize;i++,j++)
    cop_(j) = aJTP.positions_[i];
  sotDEBUGOUT(5);
  return dummy;
}

sot::MatrixHomogeneous 
SotJointTrajectoryEntity::
XYZThetaToMatrixHomogeneous (const ml::Vector& xyztheta)
{
  assert (xyztheta.size () == 4);
  ml::Vector t (3);
  t (0) = xyztheta (0);
  t (1) = xyztheta (1);
  t (2) = xyztheta (2);


  jrlMathTools::Angle theta (xyztheta (3));

  sot::VectorRollPitchYaw vR;
  vR (2) = theta.value ();
  sot::MatrixRotation R;
  vR.toMatrix (R);
  sot::MatrixHomogeneous res;
  res.buildFrom (R, t);
  return res;
}

ml::Vector &SotJointTrajectoryEntity::
getNextPosition(ml::Vector &pos,
                const int & time)
{
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  pos = pose_;
  sotDEBUGOUT(5);
  return pos;
}
   
ml::Vector &SotJointTrajectoryEntity::
getNextCoM(ml::Vector &com,
                const int & time)
{
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  com = com_;
  sotDEBUGOUT(5);
  return com;
}

ml::Vector &SotJointTrajectoryEntity::
getNextCoP(ml::Vector &cop,
                const int & time)
{
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  cop = cop_;
  sotDEBUGOUT(5);
  return cop;
}

sot::MatrixHomogeneous &SotJointTrajectoryEntity::
getNextWaist(sot::MatrixHomogeneous &waist,
                const int & time)
{
  sotDEBUGIN(5);
  OneStepOfUpdateS(time);
  waist = waist_;
  sotDEBUGOUT(5);
  return waist_;
}

void SotJointTrajectoryEntity::
loadFile(const std::string &)
{
  sotDEBUGIN(5);
  //TODO  
  sotDEBUGOUT(5);
}

void SotJointTrajectoryEntity::
display(std::ostream &os) const
{
  sotDEBUGIN(5);
  os << this;
  sotDEBUGOUT(5);
}
