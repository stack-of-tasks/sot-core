/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/debug.hh>
#include <sot/core/matrix-svd.hh>
#include <sot/core/memory-task-sot.hh>
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

MemoryTaskSOT::MemoryTaskSOT(const Matrix::Index nJ, const Matrix::Index mJ)
    : kernel(NULL, 0, 0) {
  initMemory(nJ, mJ);
}

void MemoryTaskSOT::initMemory(const Matrix::Index nJ, const Matrix::Index mJ) {
  err.resize(nJ);
  tmpTask.resize(nJ);
  tmpVar.resize(mJ);
  tmpControl.resize(mJ);
  Jt.resize(nJ, mJ);

  JK.resize(nJ, mJ);

  svd = SVD_t(nJ, mJ, Eigen::ComputeThinU | Eigen::ComputeFullV);
  // If the constraint is well conditioned, the kernel can be pre-allocated.
  if (mJ > nJ) kernelMem.resize(mJ - nJ, mJ);

  JK.setZero();
  Jt.setZero();
}

void MemoryTaskSOT::display(std::ostream& /*os*/) const {}  // TODO
