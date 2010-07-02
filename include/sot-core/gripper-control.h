/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      GripperControl.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_SOTGRIPPERCONTROL_H__
#define __SOT_SOTGRIPPERCONTROL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot-core/flags.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/sot-core-api.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

namespace dg = dynamicgraph;

class SOT_CORE_EXPORT GripperControl
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

class SOT_CORE_EXPORT GripperControlPlugin
:public dg::Entity, public GripperControl
{
 public:
  static const std::string CLASS_NAME;
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


} // namespace sot


#endif // #ifndef __SOT_SOTGRIPPERCONTROL_H__

