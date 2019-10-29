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
#include <sot/core/factory.hh>
#include <sot/core/feature-visual-point.hh>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureVisualPoint, "FeatureVisualPoint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureVisualPoint::FeatureVisualPoint(const string &pointName)
    : FeatureAbstract(pointName), L(),
      xySIN(NULL, "sotFeatureVisualPoint(" + name + ")::input(vector)::xy"),
      ZSIN(NULL, "sotFeatureVisualPoint(" + name + ")::input(double)::Z"),
      articularJacobianSIN(NULL, "sotFeatureVisualPoint(" + name +
                                     ")::input(matrix)::Jq") {
  ZSIN = 1.;

  jacobianSOUT.addDependency(xySIN);
  jacobianSOUT.addDependency(ZSIN);
  jacobianSOUT.addDependency(articularJacobianSIN);

  errorSOUT.addDependency(xySIN);
  errorSOUT.addDependency(ZSIN);

  signalRegistration(xySIN << ZSIN << articularJacobianSIN);
}

void FeatureVisualPoint::addDependenciesFromReference(void) {
  assert(isReferenceSet());
  errorSOUT.addDependency(getReference()->xySIN);
}

void FeatureVisualPoint::removeDependenciesFromReference(void) {
  assert(isReferenceSet());
  errorSOUT.removeDependency(getReference()->xySIN);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &FeatureVisualPoint::getDimension(unsigned int &dim, int time) {
  sotDEBUG(25) << "# In {" << endl;

  const Flags &fl = selectionSIN.access(time);

  dim = 0;
  if (fl(0))
    dim++;
  if (fl(1))
    dim++;

  sotDEBUG(25) << "# Out }" << endl;
  return dim;
}

/** Compute the interaction matrix from a subset of
 * the possible features.
 */
Matrix &FeatureVisualPoint::computeJacobian(Matrix &J, int time) {
  sotDEBUG(15) << "# In {" << endl;

  sotDEBUG(15) << "Get selection flags." << endl;
  const Flags &fl = selectionSIN(time);

  const int dim = dimensionSOUT(time);
  L.resize(dim, 6);
  unsigned int cursorL = 0;

  sotDEBUG(5) << std::endl;

  const double &Z = ZSIN(time);
  sotDEBUG(5) << xySIN(time) << std::endl;
  const double &x = xySIN(time)(0);
  const double &y = xySIN(time)(1);

  if (Z < 0) {
    throw(ExceptionFeature(ExceptionFeature::BAD_INIT,
                           "VisualPoint is behind the camera", " (Z=%.1f).",
                           Z));
  }

  if (fabs(Z) < 1e-6) {
    throw(ExceptionFeature(ExceptionFeature::BAD_INIT,
                           "VisualPoint Z coordinates is null", " (Z=%.3f)",
                           Z));
  }

  if (fl(0)) {
    L(cursorL, 0) = -1 / Z;
    L(cursorL, 1) = 0;
    L(cursorL, 2) = x / Z;
    L(cursorL, 3) = x * y;
    L(cursorL, 4) = -(1 + x * x);
    L(cursorL, 5) = y;

    cursorL++;
  }

  if (fl(1)) {
    L(cursorL, 0) = 0;
    L(cursorL, 1) = -1 / Z;
    L(cursorL, 2) = y / Z;
    L(cursorL, 3) = 1 + y * y;
    L(cursorL, 4) = -x * y;
    L(cursorL, 5) = -x;

    cursorL++;
  }
  sotDEBUG(15) << "L:" << endl << L << endl;
  sotDEBUG(15) << "Jq:" << endl << articularJacobianSIN(time) << endl;

  J = L * articularJacobianSIN(time);

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
Vector &FeatureVisualPoint::computeError(Vector &error, int time) {
  const Flags &fl = selectionSIN(time);
  sotDEBUGIN(15);
  error.resize(dimensionSOUT(time));
  unsigned int cursorL = 0;

  if (!isReferenceSet()) {
    throw(ExceptionFeature(ExceptionFeature::BAD_INIT,
                           "S* is not of adequate type."));
  }

  if (fl(0)) {
    error(cursorL++) = xySIN(time)(0) - getReference()->xySIN(time)(0);
  }
  if (fl(1)) {
    error(cursorL++) = xySIN(time)(1) - getReference()->xySIN(time)(1);
  }

  sotDEBUGOUT(15);
  return error;
}

void FeatureVisualPoint::display(std::ostream &os) const {
  os << "VisualPoint <" << name << ">:";

  try {
    const Vector &xy = xySIN.accessCopy();
    const Flags &fl = selectionSIN.accessCopy();
    if (fl(0))
      os << " x=" << xy(0);
    if (fl(1))
      os << " y=" << xy(1);
  } catch (ExceptionAbstract e) {
    os << " XY or select not set.";
  }

  try {
    const double &z = ZSIN.accessCopy();
    os << " Z=" << z << " ";
  } catch (ExceptionAbstract e) {
    os << " Z not set.";
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
