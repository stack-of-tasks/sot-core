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
#include <sot/core/feature-generic.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureGeneric, "FeatureGeneric");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureGeneric::FeatureGeneric(const string &pointName)
    : FeatureAbstract(pointName),
      dimensionDefault(0),
      errorSIN(NULL, "sotFeatureGeneric(" + name + ")::input(vector)::errorIN"),
      jacobianSIN(NULL,
                  "sotFeatureGeneric(" + name + ")::input(matrix)::jacobianIN")

{
  jacobianSOUT.addDependency(jacobianSIN);
  errorSOUT.addDependency(errorSIN);

  signalRegistration(errorSIN << jacobianSIN << errordotSIN << errordotSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void FeatureGeneric::addDependenciesFromReference(void) {
  assert(SP::isReferenceSet());
  errorSOUT.addDependency(getReference()->errorSIN);
  errordotSOUT.addDependency(getReference()->errordotSIN);
}

void FeatureGeneric::removeDependenciesFromReference(void) {
  assert(SP::isReferenceSet());
  errorSOUT.removeDependency(getReference()->errorSIN);
  errordotSOUT.removeDependency(getReference()->errordotSIN);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &FeatureGeneric::getDimension(unsigned int &dim, int time) {
  sotDEBUG(25) << "# In {" << endl;

  const Flags &fl = selectionSIN.access(time);

  if (dimensionDefault == 0) dimensionDefault = errorSIN.access(time).size();

  dim = 0;
  for (unsigned int i = 0; i < dimensionDefault; ++i)
    if (fl(i)) dim++;

  sotDEBUG(25) << "# Out }" << endl;
  return dim;
}

Vector &FeatureGeneric::computeError(Vector &res, int time) {
  const Vector &err = errorSIN.access(time);
  const Flags &fl = selectionSIN.access(time);
  const int &dim = dimensionSOUT(time);

  unsigned int curr = 0;
  res.resize(dim);
  if (err.size() < dim) {
    SOT_THROW ExceptionFeature(
        ExceptionFeature::UNCOMPATIBLE_SIZE,
        "Error: dimension uncompatible with des->errorIN size."
        " (while considering feature <%s>).",
        getName().c_str());
  }

  sotDEBUG(15) << "Err = " << err;
  sotDEBUG(25) << "Dim = " << dim << endl;

  if (isReferenceSet()) {
    const Vector &errDes = getReference()->errorSIN(time);
    sotDEBUG(15) << "Err* = " << errDes;
    if (errDes.size() < dim) {
      SOT_THROW ExceptionFeature(
          ExceptionFeature::UNCOMPATIBLE_SIZE,
          "Error: dimension uncompatible with des->errorIN size."
          " (while considering feature <%s>).",
          getName().c_str());
    }

    for (int i = 0; i < err.size(); ++i)
      if (fl(i))
        if (fl(i)) res(curr++) = err(i) - errDes(i);
  } else {
    for (int i = 0; i < err.size(); ++i)
      if (fl(i)) res(curr++) = err(i);
  }

  return res;
}

Matrix &FeatureGeneric::computeJacobian(Matrix &res, int time) {
  sotDEBUGIN(15);

  const Matrix &Jac = jacobianSIN.access(time);
  const Flags &fl = selectionSIN.access(time);
  const unsigned int &dim = dimensionSOUT(time);

  unsigned int curr = 0;
  res.resize(dim, Jac.cols());

  for (unsigned int i = 0; curr < dim; ++i)
    if (fl(i)) {
      for (int j = 0; j < Jac.cols(); ++j) res(curr, j) = Jac(i, j);
      curr++;
    }

  sotDEBUGOUT(15);
  return res;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void FeatureGeneric::display(std::ostream &os) const {
  os << "Generic <" << name << ">: " << std::endl;

  try {
    os << "  error= " << errorSIN.accessCopy() << endl
       << "  J    = " << jacobianSIN.accessCopy() << endl;
  } catch (ExceptionSignal &e) {
    os << e.what();
  }
}
