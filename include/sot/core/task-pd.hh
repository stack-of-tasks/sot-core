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

#ifndef __SOT_TASK_PD_H__
#define __SOT_TASK_PD_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <sot/core/task.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (task_pd_EXPORTS)
#    define SOTTASKPD_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKPD_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKPD_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTTASKPD_EXPORT TaskPD
: public Task
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  dg::Vector previousError;
  double beta;

 public:
  TaskPD( const std::string& n );


  /* --- COMPUTATION --- */
  dg::Vector& computeErrorDot( dg::Vector& error,int time );
  VectorMultiBound& computeTaskModif( VectorMultiBound& error,int time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalTimeDependent< dg::Vector,int > errorDotSOUT;
  dg::SignalPtr< dg::Vector,int > errorDotSIN;

  /* --- PARAMS --- */
  void initCommand( void );

};

} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_TASK_PD_H__ */


