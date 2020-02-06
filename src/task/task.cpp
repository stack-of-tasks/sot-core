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

/* SOT */
#include <sot/core/debug.hh>
#include <sot/core/task.hh>

#include "../src/task/task-command.h"
#include <dynamic-graph/all-commands.h>
#include <sot/core/pool.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Task, "Task");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Task::Task(const std::string &n)
    : TaskAbstract(n), featureList(), withDerivative(false),
      controlGainSIN(NULL, "sotTask(" + n + ")::input(double)::controlGain"),
      dampingGainSINOUT(NULL, "sotTask(" + n + ")::in/output(double)::damping")
      // TODO As far as I understand, this is not used in this class.
      ,
      controlSelectionSIN(NULL,
                          "sotTask(" + n + ")::input(flag)::controlSelec"),
      errorSOUT(boost::bind(&Task::computeError, this, _1, _2), sotNOSIGNAL,
                "sotTask(" + n + ")::output(vector)::error"),
      errorTimeDerivativeSOUT(
          boost::bind(&Task::computeErrorTimeDerivative, this, _1, _2),
          errorSOUT,
          "sotTask(" + n + ")::output(vector)::errorTimeDerivative") {
  taskSOUT.setFunction(
      boost::bind(&Task::computeTaskExponentialDecrease, this, _1, _2));
  jacobianSOUT.setFunction(boost::bind(&Task::computeJacobian, this, _1, _2));

  taskSOUT.addDependency(controlGainSIN);
  taskSOUT.addDependency(errorSOUT);
  taskSOUT.addDependency(errorTimeDerivativeSOUT);

  jacobianSOUT.addDependency(controlSelectionSIN);

  controlSelectionSIN = true;

  signalRegistration(controlGainSIN << dampingGainSINOUT << controlSelectionSIN
                                    << errorSOUT << errorTimeDerivativeSOUT);

  initCommands();
}

void Task::initCommands(void) {
  using namespace dynamicgraph::command;
  //
  // Commands
  //
  std::string docstring;
  // AddFeature
  docstring = "    \n"
              "    Add a feature to the task\n"
              "    \n"
              "      Input:\n"
              "        - name of the feature\n"
              "    \n";
  addCommand("add",
             makeCommandVoid1(*this, &Task::addFeatureFromName, docstring));

  addCommand("setWithDerivative",
             makeDirectSetter(*this, &withDerivative,
                              docDirectSetter("withDerivative", "bool")));
  addCommand("getWithDerivative",
             makeDirectGetter(*this, &withDerivative,
                              docDirectGetter("withDerivative", "bool")));
  // ClearFeatureList
  docstring = "    \n"
              "    Clear the list of features of the task\n"
              "    \n";

  addCommand("clear",
             makeCommandVoid0(*this, &Task::clearFeatureList, docstring));
  // List features
  docstring = "    \n"
              "    Returns the list of features of the task\n"
              "    \n";

  addCommand("list", new command::task::ListFeatures(*this, docstring));
}

void Task::addFeature(FeatureAbstract &s) {
  featureList.push_back(&s);
  jacobianSOUT.addDependency(s.jacobianSOUT);
  errorSOUT.addDependency(s.errorSOUT);
  errorTimeDerivativeSOUT.addDependency(s.getErrorDot());
}

void Task::addFeatureFromName(const std::string &featureName) {
  FeatureAbstract &feature =
      PoolStorage::getInstance()->getFeature(featureName);
  addFeature(feature);
}

void Task::clearFeatureList(void) {

  for (FeatureList_t::iterator iter = featureList.begin();
       iter != featureList.end(); ++iter) {
    FeatureAbstract &s = **iter;
    jacobianSOUT.removeDependency(s.jacobianSOUT);
    errorSOUT.removeDependency(s.errorSOUT);
    errorTimeDerivativeSOUT.removeDependency(s.getErrorDot());
  }

  featureList.clear();
}

void Task::setControlSelection(const Flags &act) { controlSelectionSIN = act; }
void Task::addControlSelection(const Flags &act) {
  Flags fl = controlSelectionSIN.accessCopy();
  fl &= act;
  controlSelectionSIN = fl;
}
void Task::clearControlSelection(void) { controlSelectionSIN = Flags(false); }

void Task::setWithDerivative(const bool &s) { withDerivative = s; }
bool Task::getWithDerivative(void) { return withDerivative; }

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

dynamicgraph::Vector &Task::computeError(dynamicgraph::Vector &error,
                                         int time) {
  sotDEBUG(15) << "# In " << getName() << " {" << endl;

  if (featureList.empty()) {
    throw(ExceptionTask(ExceptionTask::EMPTY_LIST, "Empty feature list"));
  }

  try {
    /* The vector dimensions are not known before the affectation loop.
     * They thus should be allocated on the flight, in the loop.
     * The first assumption is that the size has not changed. A double
     * reallocation (realloc(dim*2)) is done if necessary. In particulary,
     * [log_2(dim)+1] reallocations are done for the first error computation.
     * If the allocated size is too large, a correction is done after the loop.
     * The algotithmic cost is linear in affectation, logarthmic in allocation
     * numbers and linear in allocation size.
     * No assumptions are made concerning size of each vector: they are
     * not said equal, and could be different.
     */

    /* First assumption: vector dimensions have not changed. If 0, they are
     * initialized to dim 1.*/
    dynamicgraph::Vector::Index dimError = error.size();
    if (0 == dimError) {
      dimError = 1;
      error.resize(dimError);
      error.setZero();
    }

    dynamicgraph::Vector vectTmp;
    int cursorError = 0;

    /* For each cell of the list, recopy value of s, s_star and error. */
    for (FeatureList_t::iterator iter = featureList.begin();
         iter != featureList.end(); ++iter) {
      FeatureAbstract &feature = **iter;

      /* Get s, and store it in the s vector. */
      sotDEBUG(45) << "Feature <" << feature.getName() << ">." << std::endl;
      const dynamicgraph::Vector &partialError = feature.errorSOUT(time);

      const dynamicgraph::Vector::Index dim = partialError.size();
      while (cursorError + dim > dimError) // DEBUG It was >=
      {
        dimError *= 2;
        error.resize(dimError);
        error.setZero();
      }

      for (int k = 0; k < dim; ++k) {
        error(cursorError++) = partialError(k);
      }
      sotDEBUG(35) << "feature: " << partialError << std::endl;
      sotDEBUG(35) << "error: " << error << std::endl;
    }

    /* If too much memory has been allocated, resize. */
    error.conservativeResize(cursorError);
  } catch SOT_RETHROW;

  sotDEBUG(35) << "error_final: " << error << std::endl;
  sotDEBUG(15) << "# Out }" << endl;
  return error;
}

dynamicgraph::Vector &
Task::computeErrorTimeDerivative(dynamicgraph::Vector &res, int time) {
  res.resize(errorSOUT(time).size());
  dynamicgraph::Vector::Index cursor = 0;

  for (FeatureList_t::iterator iter = featureList.begin();
       iter != featureList.end(); ++iter) {
    FeatureAbstract &feature = **iter;

    const dynamicgraph::Vector &partialErrorDot = feature.getErrorDot()(time);
    const dynamicgraph::Vector::Index dim = partialErrorDot.size();
    res.segment(cursor, dim) = partialErrorDot;
    cursor += dim;
  }

  return res;
}

VectorMultiBound &
Task::computeTaskExponentialDecrease(VectorMultiBound &errorRef, int time) {
  sotDEBUG(15) << "# In {" << endl;
  const dynamicgraph::Vector &errSingleBound = errorSOUT(time);
  const double &gain = controlGainSIN(time);
  errorRef.resize(errSingleBound.size());

  for (unsigned int i = 0; i < errorRef.size(); ++i)
    errorRef[i] = -errSingleBound(i) * gain;

  if (withDerivative) {
    const dynamicgraph::Vector &de = errorTimeDerivativeSOUT(time);
    for (unsigned int i = 0; i < errorRef.size(); ++i)
      errorRef[i] = errorRef[i].getSingleBound() - de(i);
  }

  sotDEBUG(15) << "# Out }" << endl;
  return errorRef;
}

dynamicgraph::Matrix &Task::computeJacobian(dynamicgraph::Matrix &J, int time) {
  sotDEBUG(15) << "# In {" << endl;

  if (featureList.empty()) {
    throw(ExceptionTask(ExceptionTask::EMPTY_LIST, "Empty feature list"));
  }

  try {
    dynamicgraph::Matrix::Index dimJ = J.rows();
    dynamicgraph::Matrix::Index nbc = J.cols();
    if (0 == dimJ) {
      dimJ = 1;
      J.resize(dimJ, nbc);
    }

    dynamicgraph::Matrix::Index cursorJ = 0;
    // const Flags& selection = controlSelectionSIN(time);

    /* For each cell of the list, recopy value of s, s_star and error. */
    for (FeatureList_t::iterator iter = featureList.begin();
         iter != featureList.end(); ++iter) {
      FeatureAbstract &feature = **iter;
      sotDEBUG(25) << "Feature <" << feature.getName() << ">" << endl;

      /* Get s, and store it in the s vector. */
      const dynamicgraph::Matrix &partialJacobian = feature.jacobianSOUT(time);
      const dynamicgraph::Matrix::Index nbr = partialJacobian.rows();
      sotDEBUG(25) << "Jp =" << endl << partialJacobian << endl;

      if (0 == nbc) {
        nbc = partialJacobian.cols();
        J.resize(nbc, dimJ);
      } else if (partialJacobian.cols() != nbc)
        throw ExceptionTask(
            ExceptionTask::NON_ADEQUATE_FEATURES,
            "Features from the list don't have compatible-size jacobians.");

      while (cursorJ + nbr >= dimJ) {
        dimJ *= 2;
        J.conservativeResize(dimJ, nbc);
      }
      // TODO If controlSelectionSIN is really to be removed,
      // then the following loop is equivalent to:
      // J.middleRows (cursorJ, nbr) = partialJacobian;
      for (int kc = 0; kc < nbc; ++kc) {
        // 	  if( selection(kc) )
        for (unsigned int k = 0; k < nbr; ++k) {
          J(cursorJ + k, kc) = partialJacobian(k, kc);
        }
        // 	  else
        // 	    for( unsigned int k=0;k<nbr;++k ) J(cursorJ+k,kc) = 0.;
      }
      cursorJ += nbr;
    }

    /* If too much memory has been allocated, resize. */
    J.conservativeResize(cursorJ, nbc);
  } catch SOT_RETHROW;

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void Task::display(std::ostream &os) const {
  os << "Task " << name << ": " << endl;
  os << "--- LIST ---  " << std::endl;

  for (FeatureList_t::const_iterator iter = featureList.begin();
       iter != featureList.end(); ++iter) {
    os << "-> " << (*iter)->getName() << endl;
  }
}

std::ostream &Task::writeGraph(std::ostream &os) const {
  FeatureList_t::const_iterator itFeatureAbstract;
  itFeatureAbstract = featureList.begin();
  while (itFeatureAbstract != featureList.end()) {
    os << "\t\"" << (*itFeatureAbstract)->getName() << "\" -> \"" << getName()
       << "\"" << endl;
    itFeatureAbstract++;
  }
  return os;
}
