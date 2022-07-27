/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/factory.hh>
#include <sot/core/feature-vector3.hh>
#include <sot/core/matrix-geometry.hh>

using namespace dynamicgraph::sot;
using namespace std;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureVector3, "FeatureVector3");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureVector3::FeatureVector3(const string &pointName)
    : FeatureAbstract(pointName),
      vectorSIN(NULL,
                "sotFeatureVector3(" + name + ")::input(vector3)::vector"),
      positionSIN(
          NULL, "sotFeaturePoint6d(" + name + ")::input(matrixHomo)::position"),
      articularJacobianSIN(
          NULL, "sotFeatureVector3(" + name + ")::input(matrix)::Jq"),
      positionRefSIN(
          NULL, "sotFeatureVector3(" + name + ")::input(vector)::positionRef") {
  jacobianSOUT.addDependency(positionSIN);
  jacobianSOUT.addDependency(articularJacobianSIN);

  errorSOUT.addDependency(vectorSIN);
  errorSOUT.addDependency(positionSIN);
  errorSOUT.addDependency(positionRefSIN);

  signalRegistration(vectorSIN << positionSIN << articularJacobianSIN
                               << positionRefSIN);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &FeatureVector3::getDimension(unsigned int &dim, int /*time*/) {
  sotDEBUG(25) << "# In {" << endl;

  return dim = 3;
}

/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */
Matrix &FeatureVector3::computeJacobian(Matrix &J, int time) {
  sotDEBUG(15) << "# In {" << endl;

  const Matrix &Jq = articularJacobianSIN(time);
  const Vector &vect = vectorSIN(time);
  const MatrixHomogeneous &M = positionSIN(time);
  MatrixRotation R;
  R = M.linear();

  Matrix Skew(3, 3);
  Skew(0, 0) = 0;
  Skew(0, 1) = -vect(2);
  Skew(0, 2) = vect(1);
  Skew(1, 0) = vect(2);
  Skew(1, 1) = 0;
  Skew(1, 2) = -vect(0);
  Skew(2, 0) = -vect(1);
  Skew(2, 1) = vect(0);
  Skew(2, 2) = 0;

  Matrix RSk(3, 3);
  RSk = R * Skew;

  J.resize(3, Jq.cols());
  for (unsigned int i = 0; i < 3; ++i)
    for (int j = 0; j < Jq.cols(); ++j) {
      J(i, j) = 0;
      for (unsigned int k = 0; k < 3; ++k) {
        J(i, j) -= RSk(i, k) * Jq(k + 3, j);
      }
    }

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

/** Compute the error between two visual features from a subset
 *a the possible features.
 */
Vector &FeatureVector3::computeError(Vector &Mvect3, int time) {
  sotDEBUGIN(15);

  const MatrixHomogeneous &M = positionSIN(time);
  const Vector &vect = vectorSIN(time);
  const Vector &vectdes = positionRefSIN(time);

  sotDEBUG(15) << "M = " << M << std::endl;
  sotDEBUG(15) << "v = " << vect << std::endl;
  sotDEBUG(15) << "vd = " << vectdes << std::endl;

  MatrixRotation R;
  R = M.linear();
  Mvect3.resize(3);
  Mvect3 = R * vect;
  Mvect3 -= vectdes;

  sotDEBUGOUT(15);
  return Mvect3;
}

void FeatureVector3::display(std::ostream &os) const {
  os << "Vector3 <" << name << ">";
}
