/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 * Nicolas Mansard
 * Joseph Mirabel
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
  namespace sot {
    template< typename TypeRef >
    struct TypeNameHelper
    {
      static const std::string typeName;
    };
    template< typename TypeRef >
    const std::string TypeNameHelper<TypeRef>::typeName = "unspecified";

#define ADD_KNOWN_TYPE( typeid ) \
    template<>const std::string TypeNameHelper<typeid>::typeName = #typeid

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
