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

#ifndef __SOT_TASKCONTI_H__
#define __SOT_TASKCONTI_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* STD */
#include <string>

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/flags.hh>
#include <sot/core/task.hh>

#include <sot/core/exception-task.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_conti_EXPORTS)
#    define SOTTASKCONTI_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKCONTI_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKCONTI_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTTASKCONTI_EXPORT TaskConti
: public Task
{
 protected:
  enum TimeRefValues
    {
      TIME_REF_UNSIGNIFICANT = -1
      ,TIME_REF_TO_BE_SET =-2
    };


  int timeRef;
  double mu;
  dg::Vector q0;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  TaskConti( const std::string& n );

  void referenceTime( const unsigned int & t ) { timeRef = t; }
  const int &  referenceTime( void ) { return timeRef; }

  /* --- COMPUTATION --- */
  VectorMultiBound& computeContiDesiredVelocity( VectorMultiBound &task,
                                                    const int & time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< dg::Vector,int > controlPrevSIN;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display( std::ostream& os ) const;
};

} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_TASKCONTI_H__ */


