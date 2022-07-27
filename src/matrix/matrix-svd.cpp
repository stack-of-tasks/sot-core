// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)

#include <sot/core/debug.hh>
#include <sot/core/matrix-svd.hh>

namespace dynamicgraph {
using Eigen::ComputeFullV;
using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

void pseudoInverse(Matrix &_inputMatrix, Matrix &_inverseMatrix,
                   const double threshold) {
  SVD_t svd(_inputMatrix, ComputeThinU | ComputeThinV);
  SVD_t::SingularValuesType m_singularValues = svd.singularValues();
  SVD_t::SingularValuesType singularValues_inv;
  singularValues_inv.resizeLike(m_singularValues);
  for (long i = 0; i < m_singularValues.size(); ++i) {
    if (m_singularValues(i) > threshold)
      singularValues_inv(i) = 1.0 / m_singularValues(i);
    else
      singularValues_inv(i) = 0;
  }
  _inverseMatrix = (svd.matrixV() * singularValues_inv.asDiagonal() *
                    svd.matrixU().transpose());
}

void dampedInverse(const SVD_t &svd, Matrix &_inverseMatrix,
                   const double threshold) {
  typedef SVD_t::SingularValuesType SV_t;
  Eigen::ArrayWrapper<const SV_t> sigmas(svd.singularValues());

  SV_t sv_inv(sigmas / (sigmas.cwiseAbs2() + threshold * threshold));
  const Matrix::Index m = sv_inv.size();

  _inverseMatrix.noalias() = (svd.matrixV().leftCols(m) * sv_inv.asDiagonal() *
                              svd.matrixU().leftCols(m).transpose());
}

void dampedInverse(const Matrix &_inputMatrix, Matrix &_inverseMatrix,
                   Matrix &Uref, Vector &Sref, Matrix &Vref,
                   const double threshold) {
  SVD_t svd(_inputMatrix, ComputeThinU | ComputeThinV);

  dampedInverse(svd, _inverseMatrix, threshold);

  Uref = svd.matrixU();
  Vref = svd.matrixV();
  Sref = svd.singularValues();
}

void dampedInverse(const Matrix &_inputMatrix, Matrix &_inverseMatrix,
                   const double threshold) {
  SVD_t svd(_inputMatrix, ComputeThinU | ComputeFullV);
  dampedInverse(svd, _inverseMatrix, threshold);
}

}  // namespace dynamicgraph
