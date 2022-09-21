/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef DYNAMICGRAPH_SOT_VECTOR_CONSTANT_H
#define DYNAMICGRAPH_SOT_VECTOR_CONSTANT_H

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {
namespace sot {

namespace command {
namespace vectorConstant {
class Resize;
}
}  // namespace command

class VectorConstant : public Entity {
  friend class command::vectorConstant::Resize;
  int rows;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  VectorConstant(const std::string &name);

  virtual ~VectorConstant(void) {}

  SignalTimeDependent<dynamicgraph::Vector, int> SOUT;

  /// \brief Set value of vector (and therefore of output signal)
  void setValue(const dynamicgraph::Vector &inValue);
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // DYNAMICGRAPH_SOT_VECTOR_CONSTANT_H
