// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)

#include <sot/core/switch.hh>

#include <dynamic-graph/factory.h>

#include "type-name-helper.hh"

namespace dynamicgraph {
namespace sot {
template <typename Tin, typename Tout, typename Time>
std::string VariadicAbstract<Tin, Tout, Time>::getTypeInName(void) {
  return TypeNameHelper<Tin>::typeName;
}
template <typename Tin, typename Tout, typename Time>
std::string VariadicAbstract<Tin, Tout, Time>::getTypeOutName(void) {
  return TypeNameHelper<Tout>::typeName;
}

typedef Switch<Vector, int> SwitchVector;
template <> DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SwitchVector, "SwitchVector");

typedef Switch<bool, int> SwitchBool;
template <> DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SwitchBool, "SwitchBoolean");

typedef Switch<MatrixHomogeneous, int> SwitchMatrixHomogeneous;
template <>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SwitchMatrixHomogeneous,
                                   "SwitchMatrixHomogeneous");
} // namespace sot
} // namespace dynamicgraph
