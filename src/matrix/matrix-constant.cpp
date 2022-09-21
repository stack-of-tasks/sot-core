/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/factory.hh>
#include <sot/core/matrix-constant.hh>

#include "../src/matrix/matrix-constant-command.h"

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MatrixConstant, "MatrixConstant");

/* --------------------------------------------------------------------- */
/* --- MATRIX ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

MatrixConstant::MatrixConstant(const std::string &name)
    : Entity(name),
      rows(0),
      cols(0),
      SOUT("sotMatrixConstant(" + name + ")::output(matrix)::sout") {
  SOUT.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);
  signalRegistration(SOUT);
  //
  // Commands

  // Resize
  std::string docstring;
  docstring =
      "    \n"
      "    Resize the matrix and set it to zero.\n"
      "      Input\n"
      "        - unsigned int: number of lines.\n"
      "        - unsigned int: number of columns.\n"
      "\n";
  addCommand("resize", new command::matrixConstant::Resize(*this, docstring));
  // set
  docstring =
      "    \n"
      "    Set value of output signal\n"
      "    \n"
      "      input:\n"
      "        - a matrix\n"
      "    \n";
  addCommand(
      "set",
      new ::dynamicgraph::command::Setter<MatrixConstant, dynamicgraph::Matrix>(
          *this, &MatrixConstant::setValue, docstring));
}

void MatrixConstant::setValue(const dynamicgraph::Matrix &inValue) {
  SOUT.setConstant(inValue);
}
