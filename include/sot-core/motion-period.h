/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      MotionPeriod.h
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


#ifndef __SOT_JOINTLIMITS_HH__
#define __SOT_JOINTLIMITS_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot-core/exception-task.h>
#include <dynamic-graph/all-signals.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/*!
  \class MotionPeriod
*/
namespace sot {

namespace dg = dynamicgraph;

class MotionPeriod
: public dg::Entity
{

 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  
  enum MotionPeriodType
    {
      MOTION_CONSTANT
      ,MOTION_SIN
      ,MOTION_COS
    };

  struct sotMotionParam
  {
    MotionPeriodType motionType;
    unsigned int period;
    unsigned int initPeriod;
    double amplitude;
    double initAmplitude;
  };

  unsigned int size;
  std::vector< sotMotionParam > motionParams;
 
  void resize( const unsigned int & size );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  dg::SignalTimeDependent< ml::Vector,int > motionSOUT;

 public:
  MotionPeriod( const std::string& name );
  virtual ~MotionPeriod( void ) {}

  ml::Vector& computeMotion( ml::Vector& res,const int& time ); 
  
  virtual void display( std::ostream& os ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
} ;

} // namespace sot

#endif // #ifndef __SOT_JOINTLIMITS_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
