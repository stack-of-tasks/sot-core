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
#include <dynamic-graph/linear-algebra.h>

#include <Eigen/SVD>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {

typedef Eigen::JacobiSVD<Matrix> SVD_t;

void pseudoInverse(Matrix &_inputMatrix, Matrix &_inverseMatrix,
                   const double threshold = 1e-6);

void dampedInverse(const SVD_t &svd, Matrix &_inverseMatrix,
                   const double threshold = 1e-6);

void dampedInverse(const Matrix &_inputMatrix, Matrix &_inverseMatrix,
                   Matrix &Uref, Vector &Sref, Matrix &Vref,
                   const double threshold = 1e-6);

void dampedInverse(const Matrix &_inputMatrix, Matrix &_inverseMatrix,
                   const double threshold = 1e-6);

}  // namespace dynamicgraph

#endif /* #ifndef __SOT_MATRIX_SVD_H__ */
