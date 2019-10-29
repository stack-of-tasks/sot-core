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
#include <sot/core/feature-joint-limits.hh>
using namespace std;

#include <../src/feature/feature-joint-limits-command.h>
#include <sot/core/factory.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureJointLimits, "FeatureJointLimits");

const double FeatureJointLimits::THRESHOLD_DEFAULT = .9;

FeatureJointLimits::FeatureJointLimits(const string &fName)
    : FeatureAbstract(fName), threshold(THRESHOLD_DEFAULT)

      ,
      jointSIN(NULL,
               "sotFeatureJointLimits(" + name + ")::input(vector)::joint"),
      upperJlSIN(NULL,
                 "sotFeatureJointLimits(" + name + ")::input(vector)::upperJl"),
      lowerJlSIN(NULL,
                 "sotFeatureJointLimits(" + name + ")::input(vector)::lowerJl"),
      widthJlSINTERN(
          boost::bind(&FeatureJointLimits::computeWidthJl, this, _1, _2),
          upperJlSIN << lowerJlSIN,
          "sotFeatureJointLimits(" + name + ")::input(vector)::widthJl") {
  errorSOUT.addDependency(jointSIN);
  errorSOUT.addDependency(upperJlSIN);
  errorSOUT.addDependency(lowerJlSIN);

  signalRegistration(jointSIN << upperJlSIN << lowerJlSIN << widthJlSINTERN);

  // Commands
  //
  std::string docstring;
  // Actuate
  docstring = "    \n"
              "    Actuate\n"
              "    \n";
  addCommand("actuate",
             new command::featureJointLimits::Actuate(*this, docstring));
}

/* --------------------------------------------------------------------- */

void FeatureJointLimits::addDependenciesFromReference(void) {}
void FeatureJointLimits::removeDependenciesFromReference(void) {}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int &FeatureJointLimits::getDimension(unsigned int &dim, int time) {
  sotDEBUG(25) << "# In {" << endl;

  const Flags &fl = selectionSIN.access(time);
  const Matrix::Index NBJL = upperJlSIN.access(time).size();

  dim = 0;
  for (Matrix::Index i = 0; i < NBJL; ++i)
    if (fl(static_cast<int>(i)))
      dim++;

  sotDEBUG(25) << "# Out }" << endl;
  return dim;
}

Vector &FeatureJointLimits::computeWidthJl(Vector &res, const int &time) {
  sotDEBUGIN(15);

  const Vector UJL = upperJlSIN.access(time);
  const Vector LJL = lowerJlSIN.access(time);
  const Vector::Index SIZE = UJL.size();
  res.resize(SIZE);

  for (Vector::Index i = 0; i < SIZE; ++i) {
    res(i) = UJL(i) - LJL(i);
  }

  sotDEBUGOUT(15);
  return res;
}

/** Compute the interaction matrix from a subset of
 * the possible features.
 */
Matrix &FeatureJointLimits::computeJacobian(Matrix &J, int time) {
  sotDEBUG(15) << "# In {" << endl;

  const unsigned int SIZE = dimensionSOUT.access(time);
  const Vector q = jointSIN.access(time);
  const Flags &fl = selectionSIN(time);
  // const unsigned int SIZE_FF=SIZE+freeFloatingSize;
  const Vector::Index SIZE_TOTAL = q.size();
  const Vector WJL = widthJlSINTERN.access(time);
  J.resize(SIZE, SIZE_TOTAL);
  J.setZero();

  unsigned int idx = 0;
  for (unsigned int i = 0; i < SIZE_TOTAL; ++i) {
    if (fl(i)) {
      if (fabs(WJL(i)) > 1e-3)
        J(idx, i) = 1 / WJL(i);
      else
        J(idx, i) = 1.;
      idx++;
    }
  }
  //   if( 0!=freeFloatingIndex )
  //     for( unsigned int i=0;i<freeFloatingIndex;++i )
  //       {
  // 	if( fabs(WJL(i))>1e-3 ) J(i,i)=1/WJL(i); else J(i,i)=1.;
  //       }

  //   if( SIZE!=freeFloatingIndex )
  //     for( unsigned int i=freeFloatingIndex;i<SIZE;++i )
  //       {
  // 	if( fabs(WJL(i))>1e-3 ) J(i,i+freeFloatingSIZE)=1/WJL(i);
  // 	else J(i,i)=1.;
  //       }

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
Vector &FeatureJointLimits::computeError(Vector &error, int time) {
  sotDEBUGIN(15);

  const Flags &fl = selectionSIN(time);
  const Vector q = jointSIN.access(time);
  const Vector UJL = upperJlSIN.access(time);
  const Vector LJL = lowerJlSIN.access(time);
  const Vector WJL = widthJlSINTERN.access(time);
  const int SIZE = dimensionSOUT.access(time);
  const Vector::Index SIZE_TOTAL = q.size();

  sotDEBUG(25) << "q = " << q << endl;
  sotDEBUG(25) << "ljl = " << LJL << endl;
  sotDEBUG(25) << "Wjl = " << WJL << endl;
  sotDEBUG(25) << "dim = " << SIZE << endl;

  assert(UJL.size() == SIZE_TOTAL);
  assert(WJL.size() == SIZE_TOTAL);
  assert(LJL.size() == SIZE_TOTAL);
  assert(SIZE <= SIZE_TOTAL);

  error.resize(SIZE);

  unsigned int parcerr = 0;
  for (int i = 0; i < SIZE_TOTAL; ++i) {
    if (fl(i)) {
      error(parcerr++) = (q(i) - LJL(i)) / WJL(i) * 2 - 1;
    }
  }

  sotDEBUGOUT(15);
  return error;
}

void FeatureJointLimits::display(std::ostream &os) const {
  os << "JointLimits <" << name << "> ... TODO";
}
