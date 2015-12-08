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

#ifndef __SOT_MATRIX_SVD_H__
#define __SOT_MATRIX_SVD_H__


/* --- Matrix --- */
#include <Eigen/SVD>
#include <dynamic-graph/linear-algebra.h>


namespace dg = dynamicgraph;
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace Eigen {

    void pseudoInverse( dg::Matrix& _inputMatrix,
			dg::Matrix& _inverseMatrix,
			const double threshold = 1e-6)
    {
      JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeThinV);
      JacobiSVD<dg::Matrix>::SingularValuesType m_singularValues=svd.singularValues();
      JacobiSVD<dg::Matrix>::SingularValuesType singularValues_inv;
      singularValues_inv.resizeLike(m_singularValues);
      for ( long i=0; i<m_singularValues.size(); ++i) {
        if ( m_singularValues(i) > threshold )
	  singularValues_inv(i)=1.0/m_singularValues(i);
	else singularValues_inv(i)=0;
      }
      _inverseMatrix = (svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose());
    }    

    void dampedInverse( dg::Matrix& _inputMatrix,
			dg::Matrix& _inverseMatrix,
			const double threshold = 1e-6,
			dg::Matrix* Uref = NULL,
			dg::Vector* Sref = NULL,
			dg::Matrix* Vref = NULL) {
      JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeThinV);
      JacobiSVD<dg::Matrix>::SingularValuesType m_singularValues=svd.singularValues();
      JacobiSVD<dg::Matrix>::SingularValuesType singularValues_inv;
      singularValues_inv.resizeLike(m_singularValues);
      for ( long i=0; i<m_singularValues.size(); ++i) {
        if ( m_singularValues(i) > threshold )
	  singularValues_inv(i)=m_singularValues(i)/(m_singularValues(i)*m_singularValues(i)+threshold*threshold);
	else singularValues_inv(i)=0;
      }
      MatrixXd svd_matrixV = svd.matrixV();
      MatrixXd svd_matrixU = svd.matrixU();
      
      _inverseMatrix = (svd_matrixV*singularValues_inv.asDiagonal()*svd_matrixU.transpose());

      if( Uref ) Uref = &svd_matrixU;
      if( Vref ) Vref = &svd_matrixV;
      if( Sref ) Sref = &singularValues_inv;
    }    


}

#endif /* #ifndef __SOT_MATRIX_SVD_H__ */
