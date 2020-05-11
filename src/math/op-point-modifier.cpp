/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/factory.h>

#include <sot/core/matrix-geometry.hh>
#include <sot/core/op-point-modifier.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(OpPointModifier, "OpPointModifier");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

OpPointModifier::OpPointModifier(const std::string &name)
    : Entity(name), jacobianSIN(NULL, "OpPointModifior(" + name +
                                          ")::input(matrix)::jacobianIN"),
      positionSIN(NULL, "OpPointModifior(" + name +
                            ")::input(matrixhomo)::positionIN"),
      jacobianSOUT(
          boost::bind(&OpPointModifier::jacobianSOUT_function, this, _1, _2),
          jacobianSIN,
          "OpPointModifior(" + name + ")::output(matrix)::jacobian"),
      positionSOUT(
          boost::bind(&OpPointModifier::positionSOUT_function, this, _1, _2),
          positionSIN,
          "OpPointModifior(" + name + ")::output(matrixhomo)::position"),
      transformation(), isEndEffector(true) {
  sotDEBUGIN(15);

  signalRegistration(jacobianSIN << positionSIN << jacobianSOUT
                                 << positionSOUT);
  {

    using namespace dynamicgraph::command;
    addCommand(
        "getTransformation",
        makeDirectGetter(*this, &transformation.matrix(),
                         docDirectGetter("transformation", "matrix 4x4 homo")));
    addCommand(
        "setTransformation",
        makeDirectSetter(*this, &transformation.matrix(),
                         docDirectSetter("dimension", "matrix 4x4 homo")));
    addCommand("getEndEffector",
               makeDirectGetter(*this, &isEndEffector,
                                docDirectGetter("end effector mode", "bool")));
    addCommand("setEndEffector",
               makeDirectSetter(*this, &isEndEffector,
                                docDirectSetter("end effector mode", "bool")));
  }

  sotDEBUGOUT(15);
}

dynamicgraph::Matrix &
OpPointModifier::jacobianSOUT_function(dynamicgraph::Matrix &res,
                                       const int &iter) {
  if (isEndEffector) {
    const dynamicgraph::Matrix &aJa = jacobianSIN(iter);
    const MatrixHomogeneous &aMb = transformation;

    MatrixTwist bVa;
    buildFrom(aMb.inverse(), bVa);
    res = bVa * aJa; // res := bJb
    return res;
  } else {
    /* Consider that the jacobian of point A in frame A is given: J  = aJa
     * and that homogenous transformation from A to B is given aMb in
     * getTransfo() and homo transfo from 0 to A is given oMa in positionSIN.
     * Then return oJb, the jacobian of point B expressed in frame O:
     *     oJb = ( oRa 0 ; 0 oRa ) * bVa * aJa
     *         = [ I skew(oAB); 0 I ] * oJa
     * with oAB = oRb bAB = oRb (-bRa aAB ) = -oRa aAB, and aAB =
     * translation(aMb).
     */

    const dynamicgraph::Matrix &oJa = jacobianSIN(iter);
    const MatrixHomogeneous &aMb = transformation;
    const MatrixHomogeneous &oMa = positionSIN(iter);
    MatrixRotation oRa;
    oRa = oMa.linear();
    dynamicgraph::Vector aAB(3);
    aAB = aMb.translation();
    dynamicgraph::Vector oAB = oRa * aAB;

    const dynamicgraph::Matrix::Index nq = oJa.cols();
    res.resize(6, oJa.cols());
    for (int j = 0; j < nq; ++j) {
      /* This is a I*Jtrans + skew*Jrot product, unrolled by hand ... */
      res(0, j) = oJa(0, j) - oAB(1) * oJa(2 + 3, j) + oAB(2) * oJa(1 + 3, j);
      res(1, j) = oJa(1, j) - oAB(2) * oJa(0 + 3, j) + oAB(0) * oJa(2 + 3, j);
      res(2, j) = oJa(2, j) - oAB(0) * oJa(1 + 3, j) + oAB(1) * oJa(0 + 3, j);
      for (int i = 0; i < 3; ++i) {
        res(i + 3, j) = oJa(i + 3, j);
      }
    }
    return res; // res := 0Jb
  }
}

MatrixHomogeneous &
OpPointModifier::positionSOUT_function(MatrixHomogeneous &res,
                                       const int &iter) {
  sotDEBUGIN(15);
  sotDEBUGIN(15) << iter << " " << positionSIN.getTime()
                 << positionSOUT.getTime() << endl;
  const MatrixHomogeneous &position = positionSIN(iter);
  res = position * transformation;
  sotDEBUGOUT(15);
  return res;
}

void OpPointModifier::setTransformation(const Eigen::Matrix4d &tr) {
  transformation.matrix() = tr;
}
const Eigen::Matrix4d &OpPointModifier::getTransformation(void) {
  return transformation.matrix();
}

/* The following function needs an access to a specific signal via
 * the pool, using the signal path <entity.signal>. this functionnality
 * is deprecated, and the following function will have to be removed
 * in a near future. A similar functionality is available using
 * the <setTransformation> mthod, bound in python.
 */
#include <dynamic-graph/pool.h>
void OpPointModifier::setTransformationBySignalName(
    std::istringstream &cmdArgs) {
  Signal<Eigen::Matrix4d, int> &sig =
      dynamic_cast<Signal<Eigen::Matrix4d, int> &>(
          PoolStorage::getInstance()->getSignal(cmdArgs));
  setTransformation(sig.accessCopy());
}
