/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
 *
 */

#ifndef VECTOR_CONSTANT_COMMAND_H
#define VECTOR_CONSTANT_COMMAND_H

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
namespace sot {
namespace command {
namespace vectorConstant {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

// Command Resize
class Resize : public Command {
 public:
  virtual ~Resize() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Resize(VectorConstant &entity, const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::UNSIGNED), docstring) {}
  virtual Value doExecute() {
    VectorConstant &vc = static_cast<VectorConstant &>(owner());
    std::vector<Value> values = getParameterValues();
    unsigned size = values[0].value();
    Vector m(Vector::Zero(size));
    vc.SOUT.setConstant(m);

    // return void
    return Value();
  }
};  // class Resize
}  // namespace vectorConstant
}  // namespace command
} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // VECTOR_CONSTANT_COMMAND_H
