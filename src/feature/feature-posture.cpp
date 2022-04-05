// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

#include <boost/assign/list_of.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <sot/core/feature-posture.hh>
#include <string>
namespace dg = ::dynamicgraph;

using dynamicgraph::sot::FeatureAbstract;

namespace dynamicgraph {
namespace sot {
using command::Command;
using command::Value;

class FeaturePosture::SelectDof : public Command {
 public:
  virtual ~SelectDof() {}
  SelectDof(FeaturePosture &entity, const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::UNSIGNED)(Value::BOOL),
                docstring) {}
  virtual Value doExecute() {
    FeaturePosture &feature = static_cast<FeaturePosture &>(owner());
    std::vector<Value> values = getParameterValues();
    unsigned int dofId = values[0].value();
    bool control = values[1].value();
    feature.selectDof(dofId, control);
    return Value();
  }
};  // class SelectDof

FeaturePosture::FeaturePosture(const std::string &name)
    : FeatureAbstract(name),
      state_(NULL, "FeaturePosture(" + name + ")::input(Vector)::state"),
      posture_(0, "FeaturePosture(" + name + ")::input(Vector)::posture"),
      postureDot_(0, "FeaturePosture(" + name + ")::input(Vector)::postureDot"),
      activeDofs_(),
      nbActiveDofs_(0) {
  signalRegistration(state_ << posture_ << postureDot_);

  errorSOUT.addDependency(state_);
  jacobianSOUT.setConstant(Matrix());

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

FeaturePosture::~FeaturePosture() {}

unsigned int &FeaturePosture::getDimension(unsigned int &res, int) {
  res = static_cast<unsigned int>(nbActiveDofs_);
  return res;
}

dg::Vector &FeaturePosture::computeError(dg::Vector &res, int t) {
  const dg::Vector &state = state_.access(t);
  const dg::Vector &posture = posture_.access(t);

  res.resize(nbActiveDofs_);
  std::size_t index = 0;
  for (std::size_t i = 0; i < activeDofs_.size(); ++i) {
    if (activeDofs_[i]) {
      res(index) = state(i) - posture(i);
      index++;
    }
  }
  return res;
}

dg::Matrix &FeaturePosture::computeJacobian(dg::Matrix &, int) {
  throw std::runtime_error(
      "jacobian signal should be constant."
      " This function should never be called");
}

dg::Vector &FeaturePosture::computeErrorDot(dg::Vector &res, int t) {
  const Vector &postureDot = postureDot_.access(t);

  res.resize(nbActiveDofs_);
  std::size_t index = 0;
  for (std::size_t i = 0; i < activeDofs_.size(); ++i) {
    if (activeDofs_[i]) res(index++) = -postureDot(i);
  }
  return res;
}

void FeaturePosture::selectDof(unsigned dofId, bool control) {
  const Vector &state = state_.accessCopy();
  const Vector &posture = posture_.accessCopy();
  std::size_t dim(state.size());

  if (dim != (std::size_t)posture.size()) {
    throw std::runtime_error("Posture and State should have same dimension.");
  }
  // If activeDof_ vector not initialized, initialize it
  if (activeDofs_.size() != dim) {
    activeDofs_ = std::vector<bool>(dim, false);
    nbActiveDofs_ = 0;
  }

  // Check that selected dof id is valid
  if (dofId >= dim) {
    std::ostringstream oss;
    oss << "dof id should less than state dimension: " << dim << ". Received "
        << dofId << ".";
    throw ExceptionAbstract(ExceptionAbstract::TOOLS, oss.str());
  }

  if (control) {
    if (!activeDofs_[dofId]) {
      activeDofs_[dofId] = true;
      nbActiveDofs_++;
    }
  } else {  // control = false
    if (activeDofs_[dofId]) {
      activeDofs_[dofId] = false;
      nbActiveDofs_--;
    }
  }
  // recompute jacobian
  Matrix J(Matrix::Zero(nbActiveDofs_, dim));

  std::size_t index = 0;
  for (std::size_t i = 0; i < activeDofs_.size(); ++i) {
    if (activeDofs_[i]) {
      J(index, i) = 1;
      index++;
    }
  }

  jacobianSOUT.setConstant(J);
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePosture, "FeaturePosture");
}  // namespace sot
}  // namespace dynamicgraph
