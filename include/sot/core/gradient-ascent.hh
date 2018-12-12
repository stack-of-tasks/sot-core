/*
 * Copyright 2018,
 * Julian Viereck
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

#ifndef __SOT_GRADIENTASCENT_H__
#define __SOT_GRADIENTASCENT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

namespace dg = ::dynamicgraph;

namespace dynamicgraph {
  namespace sot {

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (reader_EXPORTS)
#    define SOTGRADIENTASCENT_EXPORT __declspec(dllexport)
#  else
#    define SOTGRADIENTASCENT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTGRADIENTASCENT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- TRACER ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using dynamicgraph::Entity;
using dynamicgraph::SignalPtr;
using dynamicgraph::SignalTimeDependent;

class SOTGRADIENTASCENT_EXPORT GradientAscent
: public Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL();
 public:

  SignalPtr< dg::Vector,int > gradientSIN;
  SignalPtr< double,int > learningRateSIN;
  SignalTimeDependent<int,int> refresherSINTERN;
  SignalTimeDependent<dg::Vector,int> valueSOUT;

 public:
  GradientAscent( const std::string& n );
  virtual ~GradientAscent( void );

 protected:

  dg::Vector& update(dg::Vector& res, const int& inTime);

  dg::Vector value;

  double alpha;
  bool init;

};

  } /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_TRACER_H__ */


