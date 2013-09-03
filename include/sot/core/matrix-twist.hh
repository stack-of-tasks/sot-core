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

#ifndef __SOT_MATRIX_TWIST_H__
#define __SOT_MATRIX_TWIST_H__

#include "sot/core/api.hh"
#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph { namespace sot {


class MatrixHomogeneous;
class MatrixForce;

class SOT_CORE_EXPORT MatrixTwist
: public dynamicgraph::Matrix
{

 public: 

  MatrixTwist( void ) : dynamicgraph::Matrix(6,6) { setIdentity(); }
  MatrixTwist( const  MatrixTwist & m ) : dynamicgraph::Matrix(m) {}
  virtual ~MatrixTwist( void ) { }
  explicit MatrixTwist( const MatrixHomogeneous& M ) 
    : dynamicgraph::Matrix(6,6) 
    { buildFrom(M); }

  MatrixTwist& buildFrom( const MatrixHomogeneous& trans );

  MatrixTwist& operator=( const dynamicgraph::Matrix& );
  MatrixTwist&
    inverse( MatrixTwist& invMatrix ) const ;
  inline MatrixTwist inverse( void )  const 
    { MatrixTwist Ainv; return inverse(Ainv); }

  MatrixForce& transpose( MatrixForce& Vt ) const;
  MatrixForce transpose( void ) const;
 };

} /* namespace sot */} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_MATRIX_TWIST_H__ */
