/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
 *
 */

#ifndef MATRIX_CONSTANT_COMMAND_H
#define MATRIX_CONSTANT_COMMAND_H

#include <boost/assign/list_of.hpp>

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

namespace dynamicgraph {
namespace sot {
namespace command {
namespace matrixConstant {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

// Command Resize
class Resize : public Command {
public:
  virtual ~Resize() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Resize(MatrixConstant &entity, const std::string &docstring)
      : Command(entity,
                boost::assign::list_of(Value::UNSIGNED)(Value::UNSIGNED),
                docstring) {}
  virtual Value doExecute() {
    MatrixConstant &mc = static_cast<MatrixConstant &>(owner());
    std::vector<Value> values = getParameterValues();
    unsigned rows = values[0].value();
    unsigned cols = values[1].value();
    Matrix m(Matrix::Zero(rows, cols));
    mc.SOUT.setConstant(m);

    // return void
    return Value();
  }
}; // class Resize
} // namespace matrixConstant
} // namespace command
} // namespace sot
} // namespace dynamicgraph

#endif // MATRIX_CONSTANT_COMMAND_H
