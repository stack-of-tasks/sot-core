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
#include <dynamic-graph/pool.h>
#include <sot/core/constraint.hh>
#include <sot/core/debug.hh>
using namespace std;
using namespace dynamicgraph::sot;

#include <sot/core/factory.hh>

using namespace dynamicgraph;
#include "../src/task/constraint-command.h"
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Constraint, "Constraint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Constraint::Constraint(const std::string &n) : TaskAbstract(n) {
  jacobianSOUT.setFunction(
      boost::bind(&Constraint::computeJacobian, this, _1, _2));

  signalDeregistration(taskSOUT.shortName());

  //
  // Commands
  //
  std::string docstring;
  // Addjacobian
  docstring = "    \n"
              "    \n"
              "    Add a Jacobian to the constraint\n"
              "    \n"
              "      Input:\n"
              "        - name of the signal conveying the Jacobian.\n"
              "    \n";
  addCommand("addJacobian",
             new command::constraint::AddJacobian(*this, docstring));
}

void Constraint::addJacobian(Signal<dynamicgraph::Matrix, int> &sig) {
  sotDEBUGIN(15);
  jacobianList.push_back(&sig);
  jacobianSOUT.addDependency(sig);
  sotDEBUGOUT(15);
}
void Constraint::clearJacobianList(void) {

  for (JacobianList::iterator iter = jacobianList.begin();
       iter != jacobianList.end(); ++iter) {
    Signal<dynamicgraph::Matrix, int> &s = **iter;
    jacobianSOUT.removeDependency(s);
  }

  jacobianList.clear();
}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

dynamicgraph::Matrix &Constraint::computeJacobian(dynamicgraph::Matrix &J,
                                                  int time) {
  sotDEBUG(15) << "# In {" << endl;

  if (jacobianList.empty()) {
    J.resize(0, 0);
  }
  //    { throw( ExceptionTask(ExceptionTask::EMPTY_LIST,
  // 			      "Empty feature list") ) ; }

  try {
    dynamicgraph::Matrix::Index dimJ = J.rows();
    dynamicgraph::Matrix::Index nbc = J.cols();
    if (0 == dimJ) {
      // Compute the correct size
      for (JacobianList::iterator iter = jacobianList.begin();
           iter != jacobianList.end(); ++iter) {
        Signal<dynamicgraph::Matrix, int> &jacobian = **iter;
        const dynamicgraph::Matrix &partialJacobian = jacobian(time);
        dimJ += partialJacobian.rows();
        if (nbc == 0)
          nbc = partialJacobian.cols();
        else if (nbc != partialJacobian.cols()) {
          SOT_THROW ExceptionTask(ExceptionTask::NON_ADEQUATE_FEATURES,
                                  "Features from the list don't "
                                  "have compatible-size jacobians.");
        }
      }
      J.resize(dimJ, nbc);
    }

    dynamicgraph::Matrix::Index cursorJ = 0;

    /* For each cell of the list, recopy value of s, s_star and error. */
    for (JacobianList::iterator iter = jacobianList.begin();
         iter != jacobianList.end(); ++iter) {
      Signal<dynamicgraph::Matrix, int> &jacobian = **iter;

      /* Get s, and store it in the s vector. */
      const dynamicgraph::Matrix &partialJacobian = jacobian(time);
      const dynamicgraph::Matrix::Index nbr = partialJacobian.rows();

      sotDEBUG(25) << "Jp =" << endl << partialJacobian << endl;

      J.middleRows(cursorJ, nbr) = partialJacobian;
      cursorJ += nbr;
    }

  } catch SOT_RETHROW;

  sotDEBUG(15) << "# Out }" << endl;
  return J;
}

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

namespace dynamicgraph {
namespace sot {
std::ostream &operator<<(std::ostream &os, const Constraint &t) {
  return os << t.name;
}
} // namespace sot
} // namespace dynamicgraph
