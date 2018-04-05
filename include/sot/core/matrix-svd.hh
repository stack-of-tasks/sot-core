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
		    const double threshold = 1e-6)  {
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
		    dg::Matrix& Uref,
		    dg::Vector& Sref,
		    dg::Matrix& Vref,
		    const double threshold = 1e-6) {
  sotDEBUGIN(15);
  sotDEBUG(5) << "Input Matrix: "<<_inputMatrix<<std::endl;
  JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeThinV);
  JacobiSVD<dg::Matrix>::SingularValuesType m_singularValues=svd.singularValues();
  JacobiSVD<dg::Matrix>::SingularValuesType singularValues_inv;
  singularValues_inv.resizeLike(m_singularValues);
  for ( long i=0; i<m_singularValues.size(); ++i) {
    singularValues_inv(i)=m_singularValues(i)/(m_singularValues(i)*m_singularValues(i)+threshold*threshold);
  }
  dg::Matrix matrix_U(svd.matrixU());
  dg::Matrix matrix_V(svd.matrixV());
  _inverseMatrix = (matrix_V*singularValues_inv.asDiagonal()*matrix_U.transpose());
  Uref = matrix_U; Vref = matrix_V;  Sref = m_singularValues;
  
  sotDEBUGOUT(15);
}    

void dampedInverse( dg::Matrix& _inputMatrix,
		    dg::Matrix& _inverseMatrix,
		    const double threshold = 1e-6) {
  sotDEBUGIN(15);
  sotDEBUG(5) << "Input Matrix: "<<_inputMatrix<<std::endl;
  JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeThinV);
  JacobiSVD<dg::Matrix>::SingularValuesType m_singularValues=svd.singularValues();
  JacobiSVD<dg::Matrix>::SingularValuesType singularValues_inv;
  singularValues_inv.resizeLike(m_singularValues);
  for ( long i=0; i<m_singularValues.size(); ++i) {
    singularValues_inv(i)=m_singularValues(i)/(m_singularValues(i)*m_singularValues(i)+threshold*threshold);
  }
  dg::Matrix Uref(svd.matrixU());
  dg::Matrix Vref(svd.matrixV());
  
  _inverseMatrix = (Vref*singularValues_inv.asDiagonal()*Uref.transpose());
  
  sotDEBUGOUT(15);
}    




}

#endif /* #ifndef __SOT_MATRIX_SVD_H__ */
