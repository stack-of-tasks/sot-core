/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --- MATRIX ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {
namespace command {
namespace matrixConstant {
class Resize;
}
}  // namespace command

class MatrixConstant : public Entity {
  friend class command::matrixConstant::Resize;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  int rows, cols;
  double color;

  void setValue(const dynamicgraph::Matrix &inValue);

 public:
  MatrixConstant(const std::string &name);

  virtual ~MatrixConstant(void) {}

  SignalTimeDependent<dynamicgraph::Matrix, int> SOUT;
};

}  // namespace sot
}  // namespace dynamicgraph
