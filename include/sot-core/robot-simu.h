/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      RobotSimu.h
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



#ifndef __SOT_ROBOT_SIMU_HH
#define __SOT_ROBOT_SIMU_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* -- MaaL --- */
#include <MatrixAbstractLayer/boost.h>
namespace ml= maal::boost;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/periodic-call.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (robot_simu_EXPORTS)
#    define SOTROBOTSIMU_EXPORT __declspec(dllexport)
#  else  
#    define SOTROBOTSIMU_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTROBOTSIMU_EXPORT
#endif

namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTROBOTSIMU_EXPORT RobotSimu
:public dynamicgraph::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  enum ForceSignalSource
  {
    FORCE_SIGNAL_RLEG,
    FORCE_SIGNAL_LLEG,
    FORCE_SIGNAL_RARM,
    FORCE_SIGNAL_LARM
  };

 protected:
  ml::Vector state;
  
  PeriodicCall periodicCallBefore;
  PeriodicCall periodicCallAfter;
  bool withForceSignals[4];


 public:
  
  /* --- CONSTRUCTION --- */
  RobotSimu( const std::string& name );

  void setStateSize( const unsigned int size );
  void setState( const ml::Vector st );
  void increment( const double dt = 5e-2 );
  


 public: /* --- DISPLAY --- */
  virtual void display( std::ostream& os ) const;
  SOTROBOTSIMU_EXPORT friend std::ostream& operator<< ( std::ostream& os,const RobotSimu& r )
    { r.display(os); return os;}

 public: /* --- SIGNALS --- */

  dynamicgraph::SignalPtr<ml::Vector,int> controlSIN;
  //dynamicgraph::SignalPtr<MatrixRotation,int> attitudeSIN;
  dynamicgraph::SignalPtr<ml::Vector,int> attitudeSIN;
  dynamicgraph::SignalPtr<ml::Vector,int> zmpSIN;

  dynamicgraph::Signal<ml::Vector,int> stateSOUT;
  dynamicgraph::Signal<MatrixRotation,int> attitudeSOUT;
  dynamicgraph::Signal<ml::Vector,int>* forcesSOUT[4];

  dynamicgraph::Signal<ml::Vector,int> pseudoTorqueSOUT;
  dynamicgraph::Signal<ml::Vector,int> previousControlSOUT;

  /*! \brief The current state of the robot from the command viewpoint. */
  dynamicgraph::Signal<ml::Vector,int> motorcontrolSOUT;
  /*! \brief The ZMP reference send by the previous controller. */
  dynamicgraph::Signal<ml::Vector,int> ZMPPreviousControllerSOUT;

 public: /* --- COMMANDS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );

};


} // namespace sot


#endif /* #ifndef __SOT_ROBOT_SIMU_HH */




