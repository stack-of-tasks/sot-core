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

#ifndef __SOT_TIME_STAMP__HH
#define __SOT_TIME_STAMP__HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* Matrix */
#include <jrl/mal/malv2.hh>
DECLARE_MAL_NAMESPACE(ml);

/* Classes standards. */
#ifndef WIN32
#include <sys/time.h>
#else /*WIN32*/
#include <sot/core/utils-windows.hh>
#endif /*WIN32*/

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot/core/debug.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (time_stamp_EXPORTS)
#    define TimeStamp_EXPORT __declspec(dllexport)
#  else
#    define TimeStamp_EXPORT __declspec(dllimport)
#  endif
#else
#  define TimeStamp_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class TimeStamp_EXPORT TimeStamp
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  struct timeval val;
  unsigned int offsetValue;
  bool offsetSet;

 public:

  /* --- CONSTRUCTION --- */
  TimeStamp( const std::string& name );

 public: /* --- DISPLAY --- */
  virtual void display( std::ostream& os ) const;

 public: /* --- SIGNALS --- */

  /* These signals can be called several time per period, given
   * each time a different results. Useful for chronos. */
  dg::Signal<ml::Vector,int> timeSOUT;
  dg::Signal<double,int> timeDoubleSOUT;

  /* These signals can be called several time per period, but give
   * always the same results different results. Useful for synchro. */
  dg::SignalTimeDependent<ml::Vector,int> timeOnceSOUT;
  dg::SignalTimeDependent<double,int> timeOnceDoubleSOUT;


 protected: /* --- SIGNAL FUNCTIONS --- */
  ml::Vector& getTimeStamp( ml::Vector& res,const int& time );
  double& getTimeStampDouble( const ml::Vector& vect,double& res );

 public: /* --- COMMANDS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

};


} /* namespace sot */} /* namespace dynamicgraph */


#endif /* #ifndef __SOT_SOT_HH */
