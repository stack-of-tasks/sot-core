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

const std::string MemoryTaskSOT::CLASS_NAME = "MemoryTaskSOT";

MemoryTaskSOT::MemoryTaskSOT(const std::string &name, const Matrix::Index nJ,
                             const Matrix::Index mJ, const Matrix::Index ffsize)
    : Entity(name),
      jacobianInvSINOUT("sotTaskAbstract(" + name + ")::inout(matrix)::Jinv"),
      jacobianConstrainedSINOUT("sotTaskAbstract(" + name +
                                ")::inout(matrix)::JK"),
      jacobianProjectedSINOUT("sotTaskAbstract(" + name +
                              ")::inout(matrix)::Jt"),
      singularBaseImageSINOUT("sotTaskAbstract(" + name +
                              ")::inout(matrix)::V"),
      rankSINOUT("sotTaskAbstract(" + name + ")::inout(matrix)::rank") {
  signalRegistration(jacobianInvSINOUT << singularBaseImageSINOUT << rankSINOUT
                                       << jacobianConstrainedSINOUT
                                       << jacobianProjectedSINOUT);
  initMemory(nJ, mJ, ffsize, true);
}

void MemoryTaskSOT::initMemory(const Matrix::Index nJ, const Matrix::Index mJ,
                               const Matrix::Index ffsize,
                               bool atConstruction) {
  sotDEBUG(15) << "Task-mermory " << getName() << ": resize " << nJ << "x" << mJ
               << std::endl;

  Jt.resize(nJ, mJ);
  Jp.resize(mJ, nJ);
  PJp.resize(mJ, nJ);

  Jff.resize(nJ, ffsize);
  Jact.resize(nJ, mJ);

  JK.resize(nJ, mJ);

  svd = SVD_t(nJ, mJ, Eigen::ComputeThinU | Eigen::ComputeFullV);

  JK.fill(0);
  if (atConstruction) {
    Jt.setZero();
    Jp.setZero();
    PJp.setZero();
    Jff.setZero();
    Jact.setZero();
    JK.setZero();
  } else {
    Eigen::pseudoInverse(Jt, Jp);
  }
}

void MemoryTaskSOT::display(std::ostream & /*os*/) const {} // TODO
