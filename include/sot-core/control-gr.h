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

#ifndef __SOT_Control_GR_HH__
#define __SOT_Control_GR_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (control_gr_EXPORTS)
#    define ControlGR_EXPORT __declspec(dllexport)
#  else  
#    define ControlGR_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define ControlGR_EXPORT
#endif

namespace sot {

namespace dg = dynamicgraph;


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class ControlGR_EXPORT ControlGR
: public dg::Entity
{

 public: /* --- CONSTRUCTOR ---- */

  ControlGR( const std::string & name );

 public: /* --- INIT --- */

  void init( const double& step);

 public: /* --- CONSTANTS --- */

  /* Default values. */
  static const double TIME_STEP_DEFAULT;   // = 0.001

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const; 
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }


 protected: 
  
  /* Parameters of the torque-control function: 
   * tau = - A*qddot = g */
  double TimeStep;
  double _dimension;

 public:  /* --- SIGNALS --- */

  dg::SignalPtr<ml::Matrix,int> matrixASIN;
  dg::SignalPtr<ml::Vector,int> accelerationSIN;
  dg::SignalPtr<ml::Vector,int> gravitySIN;
  dg::SignalTimeDependent<ml::Vector,int> controlSOUT;

 protected:

  double& setsize(int dimension);
  ml::Vector& computeControl( ml::Vector& tau,int t );

};


} // namespace sotte

#endif // #ifndef __SOT_Control_GR_HH__
