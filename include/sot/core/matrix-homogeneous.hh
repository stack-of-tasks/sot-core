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

// initialize all matrix and vectors to NAN
#ifndef EIGEN_INITIALIZE_MATRICES_BY_NAN
#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#endif	

#include <sot/core/api.hh>
#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {
  namespace sot {
    class MatrixRotation;

    class SOT_CORE_EXPORT MatrixHomogeneous
      : public dynamicgraph::Matrix
    {
      
    public: 
      
      MatrixHomogeneous( void );
      MatrixHomogeneous( const dynamicgraph::Matrix & copy );
      virtual ~MatrixHomogeneous( void ) { }
      
      MatrixHomogeneous& buildFrom( const MatrixRotation& rot, const dynamicgraph::Vector& trans );
      // extract(ml::Matrix): outputs a *rotation* matrix extracted from a homogeneous rotation matrix
      dynamicgraph::Matrix& extract( dynamicgraph::Matrix& rot ) const;
      MatrixRotation& extract( MatrixRotation& rot ) const;
      dynamicgraph::Vector& extract( dynamicgraph::Vector& trans ) const;
      
      MatrixHomogeneous operator*(const MatrixHomogeneous& h) const;
      dynamicgraph::Vector operator*(const dynamicgraph::Vector& v1) const;
      MatrixHomogeneous& operator=( const dynamicgraph::Matrix& );
      
      MatrixHomogeneous&
	inverse( MatrixHomogeneous& invMatrix ) const ;
      inline MatrixHomogeneous inverse( void )  const 
      { MatrixHomogeneous Ainv; return inverse(Ainv); }
      
      dynamicgraph::Vector& multiply( const dynamicgraph::Vector& v1,dynamicgraph::Vector& res ) const;
      inline dynamicgraph::Vector multiply( const dynamicgraph::Vector& v1 )  const
      { dynamicgraph::Vector res; return multiply(v1,res); }
      
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef __SOT_MATRIX_HOMOGENEOUS_H__ */






