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
	feature.selectDof (dofId, control);
	return Value ();
      }
    }; // class SelectDof

    FeaturePosture::FeaturePosture (const std::string& name)
      : FeatureAbstract(name),
	state_(NULL, "FeaturePosture("+name+")::input(Vector)::state"),
	posture_(0, "FeaturePosture("+name+")::input(Vector)::posture"),
	jacobian_(),
	activeDofs_ (),
	nbActiveDofs_ (0)
    {
      signalRegistration (state_ << posture_);

      errorSOUT.addDependency (state_);

      std::string docstring;
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


    unsigned int& FeaturePosture::getDimension( unsigned int& res,int )
    {
      res = static_cast <unsigned int> (nbActiveDofs_);
      return res;
    }

    dg::Vector& FeaturePosture::computeError( dg::Vector& res, int t)
    {
      dg::Vector state = state_.access (t);
      dg::Vector posture = posture_.access (t);

      res.resize (nbActiveDofs_);
      std::size_t index=0;
      for (std::size_t i=0; i<activeDofs_.size (); ++i) {
	if (activeDofs_ [i]) {
	  res (index) = state (i) - posture (i);
	  index ++;
	}
      }
      return res;
    }

    dg::Matrix& FeaturePosture::computeJacobian( dg::Matrix& res, int )
    {
      res = jacobian_;
      return res;
    }

    dg::Vector& FeaturePosture::computeActivation( dg::Vector& res, int )
    {
      return res;
    }

    void
    FeaturePosture::selectDof (unsigned dofId, bool control)
    {
      dg::Vector state = state_.accessCopy();
      dg::Vector posture = posture_.accessCopy ();
      int dim = state.size();

      if (dim != posture.size ()) {
	throw std::runtime_error
	  ("Posture and State should have same dimension.");
      }

      // If activeDof_ vector not initialized, initialize it
      if (activeDofs_.size () != dim) {
	activeDofs_ = std::vector <bool> (dim, false);
	nbActiveDofs_ = 0;
      }

      // Check that selected dof id is valid
      if ((dofId < 6) || (dofId >= dim))
	{
	  std::ostringstream oss;
	  oss << "dof id should be more than 5 and less than state "
	    "dimension: "
	      << dim << ". Received " << dofId << ".";
	  throw ExceptionAbstract(ExceptionAbstract::TOOLS,
				  oss.str());
	}

      if (control) {
	if (!activeDofs_ [dofId]) {
	  activeDofs_ [dofId] = true;
	  nbActiveDofs_ ++;
	}
      }
      else { // control = false
	if (activeDofs_ [dofId]) {
	  activeDofs_ [dofId] = false;
	  nbActiveDofs_ --;
	}
      }
      // recompute jacobian
      jacobian_.resize (nbActiveDofs_, dim);
      jacobian_.setZero ();

      std::size_t index=0;
      for (std::size_t i=0; i<activeDofs_.size (); ++i) {
	if (activeDofs_ [i]) {
	  jacobian_ (index, i) = 1;
	  index ++;
	}
      }

    }

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePosture, "FeaturePosture");
  } // namespace sot
} // namespace dynamicgraph

