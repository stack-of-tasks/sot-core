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

#ifndef __SOT_JOINTLIMITS_HH__
#define __SOT_JOINTLIMITS_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot/core/exception-task.hh>
#include <dynamic-graph/all-signals.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (motion_period_EXPORTS)
#    define SOTMOTIONPERIOD_EXPORT __declspec(dllexport)
#  else
#    define SOTMOTIONPERIOD_EXPORT  __declspec(dllimport)
#  endif
#else
#  define SOTMOTIONPERIOD_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/*!
  \class MotionPeriod
*/
namespace dynamicgraph { namespace sot {

namespace dg = dynamicgraph;

class SOTMOTIONPERIOD_EXPORT MotionPeriod
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

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_JOINTLIMITS_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
