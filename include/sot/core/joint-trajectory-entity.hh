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

#ifndef __SOT_JOINT_TRAJECTORY_ENTITY_HH
#define __SOT_JOINT_TRAJECTORY_ENTITY_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* -- MaaL --- */
#include <jrl/mal/boost.hh>
namespace ml= maal::boost;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot/core/trajectory.hh>
#include <list>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sot_joint_trajectory_entity_EXPORTS)
#    define SOTJOINT_TRAJECTORY_ENTITY_EXPORT __declspec(dllexport)
#  else
#    define SOTJOINT_TRAJECTORY_ENTITY_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTJOINT_TRAJECTORY_ENTITY_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {


class SOTJOINT_TRAJECTORY_ENTITY_EXPORT SotJointTrajectoryEntity
:public dynamicgraph::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  // \brief Index on the point along the trajectory.
  unsigned int index_;

  // \brief Keep the starting time as an identifier of the trajectory.
  timestamp traj_timestamp_;

 public:

  /* --- CONSTRUCTION --- */
  SotJointTrajectoryEntity( const std::string& name );
  virtual ~SotJointTrajectoryEntity( void ) { }

  void loadFile( const std::string& name );

  ml::Vector& getNextPosition( ml::Vector& pos, const int& time );

 public: /* --- DISPLAY --- */
  virtual void display( std::ostream& os ) const;
  SOTJOINT_TRAJECTORY_ENTITY_EXPORT friend std::ostream& operator<< ( std::ostream& os,const SotJointTrajectoryEntity& r )
    { r.display(os); return os;}
  
 public: /* --- SIGNALS --- */

  //dynamicgraph::SignalPtr<ml::Vector,int> positionSIN;
  //dynamicgraph::SignalTimeDependant<ml::Vector,int> velocitySOUT;
  dynamicgraph::SignalTimeDependent<int,int> refresherSINTERN;
  dynamicgraph::SignalTimeDependent<ml::Vector,int> positionSOUT;
  dynamicgraph::SignalPtr<Trajectory,int> trajectorySIN;

 public: /* --- COMMANDS --- */
  virtual void commandLine( const std::string& cmdLine,
                            std::istringstream& cmdArgs,
			    std::ostream& os );
  
};


} /* namespace sot */} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_JOINT_TRAJECTORY_ENTITY_HH */




