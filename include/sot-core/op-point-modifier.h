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

#ifndef __SOT_OP_POINT_MODIFIOR_H__
#define __SOT_OP_POINT_MODIFIOR_H__

#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/debug.h>
#include <sot-core/matrix-homogeneous.h>

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (op_point_modifier_EXPORTS)
#    define SOTOPPOINTMODIFIER_EXPORT __declspec(dllexport)
#  else  
#    define SOTOPPOINTMODIFIER_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTOPPOINTMODIFIER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTOPPOINTMODIFIER_EXPORT OpPointModifier
: public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
  
  MatrixHomogeneous transformation;

 public:

  dg::SignalPtr<ml::Matrix,int> jacobianSIN;
  dg::SignalPtr<MatrixHomogeneous,int> positionSIN;
  
  dg::SignalTimeDependent<ml::Matrix,int> jacobianSOUT;
  dg::SignalTimeDependent<MatrixHomogeneous,int> positionSOUT;

public:
  OpPointModifier( const std::string& name );

  virtual ~OpPointModifier( void ){}

  ml::Matrix& computeJacobian( ml::Matrix& res,const int& time );
  MatrixHomogeneous& computePosition( MatrixHomogeneous& res,const int& time );
  void setTransformation( const MatrixHomogeneous& tr );

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};

} // namespace sot

#endif //  __SOT_OP_POINT_MODIFIOR_H__ 
