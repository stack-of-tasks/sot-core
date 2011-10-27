// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <boost/assign/list_of.hpp>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

#include "sot/core/feature-posture.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

using ::dynamicgraph::command::Setter;
using dynamicgraph::sot::FeatureAbstract;

namespace dynamicgraph {
  namespace sot {
    using command::Command;
    using command::Value;

    class FeaturePosture::SelectDof : public Command {
    public:
      virtual ~SelectDof () {}
      SelectDof(FeaturePosture& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::UNSIGNED)
		(Value::BOOL), docstring) {}
      virtual Value doExecute()
      {
	FeaturePosture& feature = static_cast<FeaturePosture&>(owner());
	std::vector<Value> values = getParameterValues();
	unsigned int dofId = values[0].value();
	bool control = values[1].value();
	ml::Vector state = feature.state_.accessCopy();
	unsigned int dim = state.size();
	ml::Matrix& jacobian = feature.jacobian_;
	// Resize matrix if necessary
	if ((jacobian.nbRows() != dim) || (jacobian.nbCols() != dim)) {
	  jacobian.resize(dim, dim);
	  jacobian.fill(0.);
	}
	// Check that selected dof id is valid
	if ((dofId < 6) || (dofId >= dim)) {
	  std::ostringstream oss;
	  oss << "dof id should be more than 5 and less than state "
	    "dimension: "
	      << dim << ". Received " << dofId << ".";
	  throw ExceptionAbstract(ExceptionAbstract::TOOLS,
				  oss.str());
	}
	if (control) {
	  jacobian(dofId, dofId) = 1.;
	}
	else {
	  jacobian(dofId, dofId) = 0.;
	}
	return Value();
      }
    }; // class SelectDof

    FeaturePosture::FeaturePosture (const std::string& name)
      : FeatureAbstract(name),
	state_(NULL, "FeaturePosture("+name+")::input(Vector)::state"),
	posture_(),
	jacobian_()
    {
      signalRegistration (state_);

      errorSOUT.addDependency (state_);

      std::string docstring;
      docstring = "    \n"
	"    \n"
	"    Set desired posture\n"
	"    \n"
	"      input:\n"
	"        - a vector\n"
	"    \n";
      addCommand ("setPosture", new Setter<FeaturePosture, ml::Vector>
		  (*this, &FeaturePosture::setPosture, docstring));
      docstring =
	"    \n"
	"    Select degree of freedom to control\n"
	"    \n"
	"      input:\n"
	"        - positive integer: rank of degree of freedom,\n"
	"        - boolean: whether to control the selected degree of "
	"freedom.\n"
	"    \n"
	"      Note: rank should be more than 5 since posture is "
	"independent\n"
	"        from freeflyer position.\n"
	"    \n";
      addCommand("selectDof", new SelectDof(*this, docstring));
    }

    FeaturePosture::~FeaturePosture ()
    {
    }

    const std::string& FeaturePosture::getClassName () const
    {
      return CLASS_NAME;
    }

    unsigned int& FeaturePosture::getDimension( unsigned int& res,int )
    {
      // Check that dimensions of state and posture fit, otherwise, return 0
      unsigned int stateDim = state_.accessCopy().size();
      unsigned int postureDim = posture_.size();

      if (postureDim != stateDim) {
	res = 0;
      } else {
	res = postureDim;
      }
      return res;
    }

    ml::Vector& FeaturePosture::computeError( ml::Vector& res, int )
    {
      // Check that dimensions of state and posture fit, otherwise, return 0
      ml::Vector state = state_.accessCopy();
      unsigned int stateDim = state.size();
      unsigned int postureDim = posture_.size();

      if (postureDim != stateDim) {
	res.resize(0);
	return res;
      }
      res.resize(postureDim);
      for (unsigned int i=0; i<postureDim; i++) {
	res(i) = state(i) - posture_(i);
      }
      return res;
    }

    ml::Matrix& FeaturePosture::computeJacobian( ml::Matrix& res, int )
    {
      // Check that dimensions of state and posture fit, otherwise, return 0
      ml::Vector state = state_.accessCopy();
      unsigned int stateDim = state.size();
      unsigned int postureDim = posture_.size();

      if (postureDim != stateDim) {
	jacobian_.resize(0,0);
      } else {
	jacobian_.resize(postureDim, postureDim);
      }
      res = jacobian_;
      return res;
    }

    ml::Vector& FeaturePosture::computeActivation( ml::Vector& res, int )
    {
      return res;
    }

    void FeaturePosture::setPosture (const ml::Vector& posture)
    {
      posture_ = posture;
    }

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePosture, "FeaturePosture");
  } // namespace sot
} // namespace dynamicgraph

