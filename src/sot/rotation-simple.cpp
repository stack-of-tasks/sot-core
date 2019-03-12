/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/rotation-simple.hh>


bool MATLAB::fullPrec = false;
MATLAB::MATLAB( const RotationSimple& Qh,const unsigned int nJ)
{

  Matrix eye = Eigen::MatrixXd::Identity(nJ,nJ);
  Qh.multiplyRight(eye);
  initFromBubMatrix(eye);
}
