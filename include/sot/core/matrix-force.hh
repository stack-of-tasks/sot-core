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

#ifndef __SOT_MATRIX_FORCE_H__
#define __SOT_MATRIX_FORCE_H__


/* --- Matrix --- */
#include <jrl/mal/boost.hh>
#include "sot/core/api.hh"
namespace ml = maal::boost;


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {


class MatrixHomogeneous;
class MatrixTwist;


class SOT_CORE_EXPORT MatrixForce
: public ml::Matrix
{

 public: 

  MatrixForce( void ) : ml::Matrix(6,6) { setIdentity(); }
  MatrixForce( const MatrixForce & m ) : ml::Matrix(m) { }
  virtual ~MatrixForce( void ) { }
  explicit MatrixForce( const MatrixHomogeneous& M ) 
    : ml::Matrix(6,6) 
    { buildFrom(M); }

  MatrixForce& buildFrom( const MatrixHomogeneous& trans );

  MatrixForce& operator=( const ml::Matrix& );
  MatrixForce&
    inverse( MatrixForce& invMatrix ) const ;
  inline MatrixForce inverse( void )  const 
    { MatrixForce Ainv; return inverse(Ainv); }

  MatrixTwist& transpose( MatrixTwist& Vt ) const;
  MatrixTwist transpose( void ) const;

 };

} /* namespace sot */} /* namespace dynamicgraph */
    

#endif /* #ifndef __SOT_MATRIX_FORCE_H__ */






