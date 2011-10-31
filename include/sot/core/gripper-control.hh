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

class SOTGRIPPERCONTROL_EXPORT GripperControl
{
 protected:

  double offset;
  static const double OFFSET_DEFAULT;
  ml::Vector factor;

 public:
  GripperControl( void );

  void computeIncrement( const ml::Vector& torques,
			 const ml::Vector& torqueLimits,
			 const ml::Vector& currentNormPos );

  static void computeNormalizedPosition( const ml::Vector& currentPos,
					 const ml::Vector& upperLim,
					 const ml::Vector& lowerLim,
					 ml::Vector& currentNormPos );
  static void computeDenormalizedPosition( const ml::Vector& currentNormPos,
					   const ml::Vector& upperLim,
					   const ml::Vector& lowerLim,
					   ml::Vector& currentPos );

  ml::Vector& computeDesiredPosition( const ml::Vector& currentPos,
				      const ml::Vector& torques,
				      const ml::Vector& upperLim,
				      const ml::Vector& lowerLim,
				      const ml::Vector& torqueLimits,
				      ml::Vector& desPos );

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

 public: /* --- SIGNAL --- */

  /* --- INPUTS --- */
  dg::SignalPtr<ml::Vector,int> positionSIN;
  dg::SignalPtr<ml::Vector,int> upperLimitSIN;
  dg::SignalPtr<ml::Vector,int> lowerLimitSIN;
  dg::SignalPtr<ml::Vector,int> torqueSIN;
  dg::SignalPtr<ml::Vector,int> torqueLimitSIN;
  dg::SignalPtr<Flags,int> selectionSIN;

  /* --- INTERMEDIARY --- */
  dg::SignalPtr<ml::Vector,int> positionFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> positionReduceSOUT;
  dg::SignalPtr<ml::Vector,int> upperLimitFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> upperLimitReduceSOUT;
  dg::SignalPtr<ml::Vector,int> lowerLimitFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> lowerLimitReduceSOUT;
  dg::SignalPtr<ml::Vector,int> torqueFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> torqueReduceSOUT;
  dg::SignalPtr<ml::Vector,int> torqueLimitFullSizeSIN;
  dg::SignalTimeDependent<ml::Vector,int> torqueLimitReduceSOUT;

  /* --- OUTPUTS --- */
  dg::SignalTimeDependent<ml::Vector,int> desiredPositionSOUT;


 public: /* --- COMMANDLINE --- */

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

};


} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_SOTGRIPPERCONTROL_H__

