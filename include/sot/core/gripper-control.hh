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

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot/core/flags.hh>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/linear-algebra.h>

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
  dg::Vector factor;

 public:
  GripperControl( void );

  void computeIncrement( const dg::Vector& torques,
			 const dg::Vector& torqueLimits,
			 const dg::Vector& currentNormPos );

  static void computeNormalizedPosition( const dg::Vector& currentPos,
					 const dg::Vector& upperLim,
					 const dg::Vector& lowerLim,
					 dg::Vector& currentNormPos );
  static void computeDenormalizedPosition( const dg::Vector& currentNormPos,
					   const dg::Vector& upperLim,
					   const dg::Vector& lowerLim,
					   dg::Vector& currentPos );

  dg::Vector& computeDesiredPosition( const dg::Vector& currentPos,
				      const dg::Vector& torques,
				      const dg::Vector& upperLim,
				      const dg::Vector& lowerLim,
				      const dg::Vector& torqueLimits,
				      dg::Vector& desPos );

  static dg::Vector& selector( const dg::Vector& fullsize,
			       const Flags& selec,
			       dg::Vector& desPos );
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
  dg::SignalPtr<dg::Vector,int> positionSIN;
  dg::SignalPtr<dg::Vector,int> upperLimitSIN;
  dg::SignalPtr<dg::Vector,int> lowerLimitSIN;
  dg::SignalPtr<dg::Vector,int> torqueSIN;
  dg::SignalPtr<dg::Vector,int> torqueLimitSIN;
  dg::SignalPtr<Flags,int> selectionSIN;

  /* --- INTERMEDIARY --- */
  dg::SignalPtr<dg::Vector,int> positionFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector,int> positionReduceSOUT;
  dg::SignalPtr<dg::Vector,int> upperLimitFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector,int> upperLimitReduceSOUT;
  dg::SignalPtr<dg::Vector,int> lowerLimitFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector,int> lowerLimitReduceSOUT;
  dg::SignalPtr<dg::Vector,int> torqueFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector,int> torqueReduceSOUT;
  dg::SignalPtr<dg::Vector,int> torqueLimitFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector,int> torqueLimitReduceSOUT;

  /* --- OUTPUTS --- */
  dg::SignalTimeDependent<dg::Vector,int> desiredPositionSOUT;


 public: /* --- COMMANDLINE --- */

  void initCommands();

  void setOffset(const double & value);
};


} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_SOTGRIPPERCONTROL_H__

