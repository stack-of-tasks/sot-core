/*
 * Copyright 2019,
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

#include <sot/core/double-constant.hh>

#include <dynamic-graph/command-setter.h>
#include <sot/core/factory.hh>

namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DoubleConstant, "DoubleConstant");

DoubleConstant::DoubleConstant(const std::string &name)
    : Entity(name), SOUT("DoubleConstant(" + name + ")::output(double)::sout") {
  SOUT.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);
  signalRegistration(SOUT);
  //
  // Commands

  // set
  std::string docstring = "    \n"
                          "    Set value of output signal\n"
                          "    \n"
                          "      input:\n"
                          "        - a double\n"
                          "    \n";
  addCommand("set", new ::dynamicgraph::command::Setter<DoubleConstant, double>(
                        *this, &DoubleConstant::setValue, docstring));
}

void DoubleConstant::setValue(const double &inValue) {
  SOUT.setConstant(inValue);
}

} // namespace sot
} // namespace dynamicgraph
