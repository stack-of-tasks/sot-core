/*
 * Copyright 2010,
 * François Bleibel,
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

#ifndef __SOT_SOTGRIPPERCONTROL_H__
#define __SOT_SOTGRIPPERCONTROL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot/core/flags.hh>
#include <dynamic-graph/all-signals.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (gripper_control_EXPORTS)
#    define SOTGRIPPERCONTROL_EXPORT __declspec(dllexport)
#  else
#    define SOTGRIPPERCONTROL_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTGRIPPERCONTROL_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {

namespace dg = dynamicgraph;

/*! \brief The goal of this entity is to ensure that the maximal torque will not
 * be exceeded during a grasping task.
 * If the maximal torque is reached, then the current position of the gripper is 
kept
 *
 */
class SOTGRIPPERCONTROL_EXPORT GripperControl
{
 protected:

  double offset;
  static const double OFFSET_DEFAULT;
  //! \brief The multiplication
  ml::Vector factor;

 public:
  GripperControl( void );

  //! \brief Computes the
  // if the torque limit is reached, the normalized position is reduced by
  // (offset)
  void computeIncrement( const ml::Vector& torques,
                         const ml::Vector& torqueLimits,
                         const ml::Vector& currentNormVel );

  //! \brief
  ml::Vector& computeDesiredPosition( const ml::Vector& currentPos,
                                      const ml::Vector& desiredPos,
                                      const ml::Vector& torques,
                                      const ml::Vector& torqueLimits,
                                      ml::Vector& referencePos );

  /*! \brief select only some of the values of the vector fullsize,
  *   based on the Flags vector.
  */

  static ml::Vector& selector( const ml::Vector& fullsize,
                               const Flags& selec,
                               ml::Vector& desPos );
};

/* --------------------------------------------------------------------- */
/* --- PLUGIN ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTGRIPPERCONTROL_EXPORT GripperControlPlugin
:public dg::Entity, public GripperControl
{
  DYNAMIC_GRAPH_ENTITY_DECL();
 public:
  bool calibrationStarted;


 public: /* --- CONSTRUCTION --- */

  GripperControlPlugin( const std::string& name );
  virtual ~GripperControlPlugin( void );

  /* --- DOCUMENTATION --- */
  virtual std::string getDocString () const;

 public: /* --- SIGNAL --- */

  /* --- INPUTS --- */
  dg::SignalPtr<ml::Vector,int> positionSIN;
  dg::SignalPtr<ml::Vector,int> positionDesSIN;
  dg::SignalPtr<ml::Vector,int> torqueSIN;
  dg::SignalPtr<ml::Vector,int> torqueLimitSIN;
  dg::SignalPtr<Flags,int> selectionSIN;

  /* --- INTERMEDIARY --- */
  dg::SignalPtr<ml::Vector,int> positionFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> positionReduceSOUT;
  dg::SignalPtr<ml::Vector,int> torqueFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> torqueReduceSOUT;
  dg::SignalPtr<ml::Vector,int> torqueLimitFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> torqueLimitReduceSOUT;

  /* --- OUTPUTS --- */
  dg::SignalTimeDependent<ml::Vector,int> desiredPositionSOUT;


 public: /* --- COMMANDLINE --- */

  void initCommands();

  void setOffset(const double & value);
};


} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_SOTGRIPPERCONTROL_H__

