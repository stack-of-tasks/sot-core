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
#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/feature-line-distance.hh>

#include <sot/core/matrix-geometry.hh>

using namespace std;

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureLineDistance, "FeatureLineDistance");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureLineDistance::FeatureLineDistance(const string &pointName)
    : FeatureAbstract(pointName),
      positionSIN(NULL, "sotFeatureLineDistance(" + name +
                            ")::input(matrixHomo)::position"),
      articularJacobianSIN(NULL, "sotFeatureLineDistance(" + name +
                                     ")::input(matrix)::Jq"),
      positionRefSIN(NULL, "sotFeatureLineDistance(" + name +
                               ")::input(vector)::positionRef"),
      vectorSIN(NULL,
                "sotFeatureVector3(" + name + ")::input(vector3)::vector"),
      lineSOUT(boost::bind(&FeatureLineDistance::computeLineCoordinates, this,
                           _1, _2),
               positionSIN << positionRefSIN,
               "sotFeatureAbstract(" + name + ")::output(vector)::line") {
  jacobianSOUT.addDependency(positionSIN);
  jacobianSOUT.addDependency(articularJacobianSIN);

  errorSOUT.addDependency(positionSIN);

  signalRegistration(positionSIN << articularJacobianSIN << positionRefSIN
                                 << lineSOUT << vectorSIN);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &FeatureLineDistance::getDimension(unsigned int &dim,
                                                int /*time*/) {
  sotDEBUG(25) << "# In {" << endl;

  return dim = 1;
}

/* --------------------------------------------------------------------- */
Vector &FeatureLineDistance::computeLineCoordinates(Vector &cood, int time) {
  sotDEBUGIN(15);

  cood.resize(6);

  /* Line coordinates */
  const MatrixHomogeneous &pos = positionSIN(time);
  const Vector &vect = vectorSIN(time);
  MatrixRotation R;
  R = pos.linear();
  Vector v(3);
  v = R * vect;

  cood(0) = pos(0, 3);
  cood(1) = pos(1, 3);
  cood(2) = pos(2, 3);
  cood(3) = v(0);
  cood(4) = v(1);
  cood(5) = v(2);

  sotDEBUGOUT(15);
  return cood;
}

/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */
Matrix &FeatureLineDistance::computeJacobian(Matrix &J, int time) {
  sotDEBUG(15) << "# In {" << endl;

  /* --- Compute the jacobian of the line coordinates --- */
  Matrix Jline;
  {
    const Matrix &Jq = articularJacobianSIN(time);

    const Vector &vect = vectorSIN(time);
    const MatrixHomogeneous &M = positionSIN(time);
    MatrixRotation R;
    R = M.linear(); // wRh

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

    Jline.resize(6, Jq.cols());
    for (unsigned int i = 0; i < 3; ++i)
      for (int j = 0; j < Jq.cols(); ++j) {
        Jline(i, j) = 0;
        Jline(i + 3, j) = 0;
        for (unsigned int k = 0; k < 3; ++k) {
          Jline(i, j) += R(i, k) * Jq(k, j);
          Jline(i + 3, j) += -RSk(i, k) * Jq(k + 3, j);
        }
      }
  }

  /* --- Compute the jacobian wrt the line coordinates --- */
  const Vector &line = lineSOUT(time);
  const double &x0 = line(0);
  const double &y0 = line(1);
  const double &z0 = line(2);
  const double &a0 = line(3);
  const double &b0 = line(4);
  const double &c0 = line(5);

  const Vector &posRef = positionRefSIN(time);
  const double &x1 = posRef(0);
  const double &y1 = posRef(1);
  const double &z1 = posRef(2);
  const double &a1 = posRef(3);
  const double &b1 = posRef(4);
  const double &c1 = posRef(5);

  /* Differential */
  const double a1_3 = a1 * a1 * a1;
  const double b1_3 = b1 * b1 * b1;
  const double c1_3 = c1 * c1 * c1;

  double K = c0 * c0 * a1 * a1 - 2 * c0 * a1 * a0 * c1 - 2 * c0 * b1 * b0 * c1 +
             c0 * c0 * b1 * b1 - 2 * b0 * a1 * a0 * b1 + b0 * b0 * a1 * a1 +
             b0 * b0 * c1 * c1 + a0 * a0 * b1 * b1 + a0 * a0 * c1 * c1;

  const double diffx0 = -b0 * c1 + c0 * b1;
  const double diffy0 = a0 * c1 - c0 * a1;
  const double diffz0 = -a0 * b1 + b0 * a1;

  const double diffa0 =
      2 * b0 * c1 * x0 * a0 * b1 * b1 + 2 * c0 * b1 * b1 * x0 * b0 * a1 +
      2 * c0 * c0 * b1 * x0 * c1 * a1 + 2 * c1 * c1 * y0 * c0 * a1 * a0 -
      2 * b0 * c1 * c1 * x0 * c0 * a1 - 2 * b0 * b0 * c1 * x0 * b1 * a1 -
      2 * c1 * c1 * y0 * c0 * b1 * b0 + 2 * b0 * b0 * c1 * x1 * b1 * a1 +
      2 * b0 * c1 * c1 * x1 * c0 * a1 - 2 * b0 * c1 * x1 * a0 * b1 * b1 -
      c1 * y0 * c0 * c0 * a1 * a1 + c1 * y0 * c0 * c0 * b1 * b1 +
      c1 * y0 * b0 * b0 * a1 * a1 - c1 * y0 * a0 * a0 * b1 * b1 +
      c1 * y1 * c0 * c0 * a1 * a1 - c1 * y1 * c0 * c0 * b1 * b1 -
      c1 * y1 * b0 * b0 * a1 * a1 + c1 * y1 * a0 * a0 * b1 * b1 -
      b1 * z0 * c0 * c0 * a1 * a1 + b1 * z0 * b0 * b0 * a1 * a1 -
      b1 * z0 * b0 * b0 * c1 * c1 + b1 * z0 * a0 * a0 * c1 * c1 +
      b1 * z1 * c0 * c0 * a1 * a1 - b1 * z1 * b0 * b0 * a1 * a1 +
      b1 * z1 * b0 * b0 * c1 * c1 - b1 * z1 * a0 * a0 * c1 * c1 +
      2 * b0 * c1_3 * x0 * a0 - 2 * b0 * c1_3 * x1 * a0 -
      2 * c0 * b1_3 * x0 * a0 + 2 * c0 * b1_3 * x1 * a0 + c1_3 * y0 * b0 * b0 -
      c1_3 * y0 * a0 * a0 - c1_3 * y1 * b0 * b0 + c1_3 * y1 * a0 * a0 -
      b1_3 * z0 * c0 * c0 + b1_3 * z0 * a0 * a0 + b1_3 * z1 * c0 * c0 -
      b1_3 * z1 * a0 * a0 - 2 * c1 * c1 * y1 * c0 * a1 * a0 +
      2 * c1 * c1 * y1 * c0 * b1 * b0 + 2 * b1 * b1 * z0 * c0 * b0 * c1 -
      2 * b1 * b1 * z0 * b0 * a1 * a0 - 2 * b1 * b1 * z1 * c0 * b0 * c1 +
      2 * b1 * b1 * z1 * b0 * a1 * a0 - 2 * c0 * b1 * x0 * a0 * c1 * c1 -
      2 * c0 * b1 * b1 * x1 * b0 * a1 - 2 * c0 * c0 * b1 * x1 * c1 * a1 +
      2 * c0 * b1 * x1 * a0 * c1 * c1 + 2 * c0 * a1 * y0 * a0 * b1 * b1 -
      2 * c0 * a1 * a1 * y0 * b1 * b0 - 2 * c0 * a1 * y1 * a0 * b1 * b1 +
      2 * c0 * a1 * a1 * y1 * b1 * b0 + 2 * b0 * a1 * a1 * z0 * c1 * c0 -
      2 * b0 * a1 * z0 * a0 * c1 * c1 - 2 * b0 * a1 * a1 * z1 * c1 * c0 +
      2 * b0 * a1 * z1 * a0 * c1 * c1;
  const double diffb0 =
      -2 * c1 * c1 * x0 * c0 * b1 * b0 + 2 * c1 * c1 * x0 * c0 * a1 * a0 -
      c1 * x0 * c0 * c0 * a1 * a1 + c1 * x0 * c0 * c0 * b1 * b1 +
      c1 * x0 * b0 * b0 * a1 * a1 - c1 * x0 * a0 * a0 * b1 * b1 +
      c1 * x1 * c0 * c0 * a1 * a1 - c1 * x1 * c0 * c0 * b1 * b1 -
      c1 * x1 * b0 * b0 * a1 * a1 + c1 * x1 * a0 * a0 * b1 * b1 +
      a1 * z0 * c0 * c0 * b1 * b1 - a1 * z0 * b0 * b0 * c1 * c1 -
      a1 * z0 * a0 * a0 * b1 * b1 + a1 * z0 * a0 * a0 * c1 * c1 -
      a1 * z1 * c0 * c0 * b1 * b1 + a1 * z1 * b0 * b0 * c1 * c1 +
      a1 * z1 * a0 * a0 * b1 * b1 - a1 * z1 * a0 * a0 * c1 * c1 -
      2 * a0 * c1_3 * y0 * b0 + 2 * a0 * c1_3 * y1 * b0 + c1_3 * x0 * b0 * b0 -
      c1_3 * x0 * a0 * a0 - c1_3 * x1 * b0 * b0 + c1_3 * x1 * a0 * a0 +
      a1_3 * z0 * c0 * c0 - 2 * c1 * c1 * x1 * c0 * a1 * a0 +
      2 * c1 * c1 * x1 * c0 * b1 * b0 - 2 * a1 * a1 * z0 * c0 * a0 * c1 +
      2 * a1 * a1 * z0 * b0 * a0 * b1 - a1_3 * z0 * b0 * b0 -
      a1_3 * z1 * c0 * c0 + a1_3 * z1 * b0 * b0 +
      2 * a1 * a1 * z1 * c0 * a0 * c1 - 2 * a1 * a1 * z1 * b0 * a0 * b1 +
      2 * c0 * b1 * b1 * x0 * a1 * a0 - 2 * c0 * b1 * x0 * b0 * a1 * a1 -
      2 * c0 * b1 * b1 * x1 * a1 * a0 + 2 * c0 * b1 * x1 * b0 * a1 * a1 +
      2 * a0 * a0 * c1 * y0 * a1 * b1 - 2 * a0 * c1 * y0 * b0 * a1 * a1 +
      2 * a0 * c1 * c1 * y0 * c0 * b1 - 2 * a0 * a0 * c1 * y1 * a1 * b1 +
      2 * a0 * c1 * y1 * b0 * a1 * a1 - 2 * a0 * c1 * c1 * y1 * c0 * b1 -
      2 * c0 * a1 * a1 * y0 * a0 * b1 + 2 * c0 * a1_3 * y0 * b0 -
      2 * c0 * a1_3 * y1 * b0 + 2 * c0 * a1 * y0 * b0 * c1 * c1 -
      2 * c0 * c0 * a1 * y0 * c1 * b1 + 2 * c0 * a1 * a1 * y1 * a0 * b1 -
      2 * c0 * a1 * y1 * b0 * c1 * c1 + 2 * c0 * c0 * a1 * y1 * c1 * b1 +
      2 * a0 * b1 * z0 * b0 * c1 * c1 - 2 * a0 * b1 * b1 * z0 * c1 * c0 -
      2 * a0 * b1 * z1 * b0 * c1 * c1 + 2 * a0 * b1 * b1 * z1 * c1 * c0;

  Matrix diffh(1, 6);
  diffh(0, 0) = diffx0 / K;
  diffh(0, 1) = diffy0 / K;
  diffh(0, 2) = diffz0 / K;
  K *= K;
  diffh(0, 3) = diffa0 / K;
  diffh(0, 4) = diffb0 / K;
  diffh(0, 5) = 0;

  /* --- Multiply Jline=dline/dq with diffh=de/dline --- */
  J.resize(1, J.cols());
  J = diffh * Jline;
  // J=Jline;

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

/** Compute the error between two visual features from a subset
 *a the possible features.
 */
Vector &FeatureLineDistance::computeError(Vector &error, int time) {
  sotDEBUGIN(15);

  /* Line coordinates */
  const Vector &line = lineSOUT(time);
  const double &x0 = line(0);
  const double &y0 = line(1);
  const double &z0 = line(2);
  const double &a0 = line(3);
  const double &b0 = line(4);
  const double &c0 = line(5);

  const Vector &posRef = positionRefSIN(time);
  const double &x1 = posRef(0);
  const double &y1 = posRef(1);
  const double &z1 = posRef(2);
  const double &a1 = posRef(3);
  const double &b1 = posRef(4);
  const double &c1 = posRef(5);

  error.resize(1);
  double K = c0 * c0 * a1 * a1 - 2 * c0 * a1 * a0 * c1 - 2 * c0 * b1 * b0 * c1 +
             c0 * c0 * b1 * b1 - 2 * b0 * a1 * a0 * b1 + b0 * b0 * a1 * a1 +
             b0 * b0 * c1 * c1 + a0 * a0 * b1 * b1 + a0 * a0 * c1 * c1;
  error(0) = (-b0 * c1 * x0 + b0 * c1 * x1 + c0 * b1 * x0 - c0 * b1 * x1 +
              a0 * c1 * y0 - a0 * c1 * y1 - c0 * a1 * y0 + c0 * a1 * y1 -
              a0 * b1 * z0 + a0 * b1 * z1 + b0 * a1 * z0 - b0 * a1 * z1) /
             K;

  /* --- DEBUG --- */
  //   error.resize(6);
  //   error=line-posRef;

  sotDEBUGOUT(15);
  return error;
}

void FeatureLineDistance::display(std::ostream &os) const {
  os << "LineDistance <" << name << ">";
}
