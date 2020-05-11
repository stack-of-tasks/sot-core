/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 * Nicolas Mansard
 * Joseph Mirabel
 *
 * CNRS/AIST
 *
 */

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
namespace sot {
template <typename TypeRef> struct TypeNameHelper {
  static const std::string typeName;
};
template <typename TypeRef>
const std::string TypeNameHelper<TypeRef>::typeName = "unspecified";

#define ADD_KNOWN_TYPE(typeid)                                                 \
  template <> const std::string TypeNameHelper<typeid>::typeName = #typeid

ADD_KNOWN_TYPE(bool);
ADD_KNOWN_TYPE(double);
ADD_KNOWN_TYPE(Vector);
ADD_KNOWN_TYPE(Matrix);
ADD_KNOWN_TYPE(MatrixRotation);
ADD_KNOWN_TYPE(MatrixTwist);
ADD_KNOWN_TYPE(MatrixHomogeneous);
ADD_KNOWN_TYPE(VectorQuaternion);
ADD_KNOWN_TYPE(VectorRollPitchYaw);

#undef ADD_KNOWN_TYPE
} /* namespace sot */
} /* namespace dynamicgraph */
