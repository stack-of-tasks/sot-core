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

#ifndef __SOT_MATRIX_HOMOGENEOUS_H__
#define __SOT_MATRIX_HOMOGENEOUS_H__


/* --- Matrix --- */
#include <jrl/mal/boost.hh>
#include <sot/core/api.hh>
namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {
  namespace sot {
    class MatrixRotation;

    class SOT_CORE_EXPORT MatrixHomogeneous
      : public ml::Matrix
    {
      
    public: 
      
      MatrixHomogeneous( void );
      MatrixHomogeneous( const ml::Matrix & copy );
      virtual ~MatrixHomogeneous( void ) { }
      
      MatrixHomogeneous& buildFrom( const MatrixRotation& rot, const ml::Vector& trans );
      // extract(ml::Matrix): outputs a *rotation* matrix extracted from a homogeneous rotation matrix
      ml::Matrix& extract( ml::Matrix& rot ) const;
      MatrixRotation& extract( MatrixRotation& rot ) const;
      ml::Vector& extract( ml::Vector& trans ) const;
      
      MatrixHomogeneous operator*(const MatrixHomogeneous& h) const;
      MatrixHomogeneous& operator=( const ml::Matrix& );
      
      MatrixHomogeneous&
	inverse( MatrixHomogeneous& invMatrix ) const ;
      inline MatrixHomogeneous inverse( void )  const 
      { MatrixHomogeneous Ainv; return inverse(Ainv); }
      
      ml::Vector& multiply( const ml::Vector& v1,ml::Vector& res ) const;
      inline ml::Vector multiply( const ml::Vector& v1 )  const
      { ml::Vector res; return multiply(v1,res); }
      
      using  ml::Matrix::multiply;
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef __SOT_MATRIX_HOMOGENEOUS_H__ */






