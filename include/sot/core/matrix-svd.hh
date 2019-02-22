/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
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
		    const double threshold = 1e-6);

void dampedInverse( const JacobiSVD <dg::Matrix>& svd,
		    dg::Matrix& _inverseMatrix,
		    const double threshold = 1e-6);

void dampedInverse( const dg::Matrix& _inputMatrix,
		    dg::Matrix& _inverseMatrix,
		    dg::Matrix& Uref,
		    dg::Vector& Sref,
		    dg::Matrix& Vref,
		    const double threshold = 1e-6);

void dampedInverse( const dg::Matrix& _inputMatrix,
		    dg::Matrix& _inverseMatrix,
		    const double threshold = 1e-6);

}

#endif /* #ifndef __SOT_MATRIX_SVD_H__ */
