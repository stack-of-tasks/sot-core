// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of sot_hpp.
// sot_hpp is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// sot_hpp is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot_hpp. If not, see <http://www.gnu.org/licenses/>.

#include <sot/core/switch.hh>

#include <dynamic-graph/factory.h>

#include "type-name-helper.hh"

namespace dynamicgraph {
  namespace sot {
    template< typename Tin, typename Tout, typename Time >
    std::string VariadicAbstract<Tin,Tout,Time>::getTypeInName (void) { return TypeNameHelper<Tin>::typeName; }
    template< typename Tin, typename Tout, typename Time >
    std::string VariadicAbstract<Tin,Tout,Time>::getTypeOutName(void) { return TypeNameHelper<Tout>::typeName; }

    typedef Switch<Vector,int> SwitchVector;
    template<>
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (SwitchVector, "SwitchVector");

    typedef Switch<bool,int> SwitchBool;
    template<>
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (SwitchBool, "SwitchBoolean");
  } // namespace sot
} // namespace dynamicgraph
