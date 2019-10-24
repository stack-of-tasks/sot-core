// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)

#include <sot/core/debug.hh>
#include <sot/core/matrix-svd.hh>

namespace Eigen {

void pseudoInverse(dg::Matrix &_inputMatrix, dg::Matrix &_inverseMatrix,
                   const double threshold) {
  JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeThinV);
  JacobiSVD<dg::Matrix>::SingularValuesType m_singularValues =
      svd.singularValues();
  JacobiSVD<dg::Matrix>::SingularValuesType singularValues_inv;
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

void dampedInverse(const JacobiSVD<dg::Matrix> &svd, dg::Matrix &_inverseMatrix,
                   const double threshold) {
  typedef JacobiSVD<dg::Matrix>::SingularValuesType SV_t;
  ArrayWrapper<const SV_t> sigmas(svd.singularValues());

  SV_t sv_inv(sigmas / (sigmas.cwiseAbs2() + threshold * threshold));
  const dg::Matrix::Index m = sv_inv.size();

  _inverseMatrix.noalias() = (svd.matrixV().leftCols(m) * sv_inv.asDiagonal() *
                              svd.matrixU().leftCols(m).transpose());
}

void dampedInverse(const dg::Matrix &_inputMatrix, dg::Matrix &_inverseMatrix,
                   dg::Matrix &Uref, dg::Vector &Sref, dg::Matrix &Vref,
                   const double threshold) {
  sotDEBUGIN(15);
  sotDEBUG(5) << "Input Matrix: " << _inputMatrix << std::endl;
  JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeThinV);

  dampedInverse(svd, _inverseMatrix, threshold);

  Uref = svd.matrixU();
  Vref = svd.matrixV();
  Sref = svd.singularValues();

  sotDEBUGOUT(15);
}

void dampedInverse(const dg::Matrix &_inputMatrix, dg::Matrix &_inverseMatrix,
                   const double threshold) {
  sotDEBUGIN(15);
  sotDEBUG(5) << "Input Matrix: " << _inputMatrix << std::endl;

  JacobiSVD<dg::Matrix> svd(_inputMatrix, ComputeThinU | ComputeFullV);
  dampedInverse(svd, _inverseMatrix, threshold);

  sotDEBUGOUT(15);
}

} // namespace Eigen
