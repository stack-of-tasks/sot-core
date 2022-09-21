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
#include <sot/core/feature-1d.hh>
using namespace std;

#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Feature1D, "Feature1D");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Feature1D::Feature1D(const string &pointName)
    : FeatureAbstract(pointName),
      errorSIN(NULL, "sotFeature1D(" + name + ")::input(vector)::errorIN"),
      jacobianSIN(NULL,
                  "sotFeature1D(" + name + ")::input(matrix)::jacobianIN") {
  jacobianSOUT.addDependency(jacobianSIN);
  errorSOUT.addDependency(errorSIN);

  signalRegistration(errorSIN << jacobianSIN);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Feature1D::addDependenciesFromReference(void) {}
void Feature1D::removeDependenciesFromReference(void) {}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &Feature1D::getDimension(unsigned int &dim, int /*time*/) {
  sotDEBUG(25) << "# In {" << endl;

  dim = 1;

  sotDEBUG(25) << "# Out }" << endl;
  return dim;
}

dynamicgraph::Vector &Feature1D::computeError(dynamicgraph::Vector &res,
                                              int time) {
  const dynamicgraph::Vector &err = errorSIN.access(time);
  res.resize(1);
  res(0) = err.dot(err) * .5;

  return res;
}

dynamicgraph::Matrix &Feature1D::computeJacobian(dynamicgraph::Matrix &res,
                                                 int time) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix &Jac = jacobianSIN.access(time);
  const dynamicgraph::Vector &err = errorSIN.access(time);

  res.resize(1, Jac.cols());
  res.fill(0);
  for (int j = 0; j < Jac.cols(); ++j)
    for (int i = 0; i < Jac.rows(); ++i) res(0, j) += err(i) * Jac(i, j);

  sotDEBUGOUT(15);
  return res;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Feature1D::display(std::ostream &os) const {
  os << "1D <" << name << ">: " << std::endl;

  try {
    os << "  error= " << errorSIN.accessCopy() << endl
       << "  J    = " << jacobianSIN.accessCopy() << endl;
  } catch (ExceptionAbstract e) {
    os << " All SIN not set.";
  }
}
