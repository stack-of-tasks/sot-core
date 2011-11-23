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

#ifndef __SOT_TASKUNILATERAL_H__
#define __SOT_TASKUNILATERAL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/malv2.hh>
DECLARE_MAL_NAMESPACE(ml);

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
#  if defined (task_unilateral_EXPORTS)
#    define SOTTASKUNILATERAL_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKUNILATERAL_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKUNILATERAL_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTTASKUNILATERAL_EXPORT TaskUnilateral
: public Task
{
 protected:
  std::list< FeatureAbstract* > featureList;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  TaskUnilateral( const std::string& n );

  /* --- COMPUTATION --- */
  VectorMultiBound& computeTaskUnilateral( VectorMultiBound& res,int time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  dg::SignalPtr< ml::Vector,int > positionSIN;
  dg::SignalPtr< ml::Vector,int > referenceInfSIN;
  dg::SignalPtr< ml::Vector,int > referenceSupSIN;
  dg::SignalPtr< double,int > dtSIN;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display( std::ostream& os ) const;
};

} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_TASKUNILATERAL_H__ */


