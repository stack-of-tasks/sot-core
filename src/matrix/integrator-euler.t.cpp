/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
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

#include <sot/core/integrator-euler.hh>
#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_EULER(sotClassType,sotSigType,sotCoefType,className)   \
  template<>                                                                                \
  std::string sotClassType<sotSigType,sotCoefType>::                                        \
  getTypeName( void ) { return #sotSigType; }                                               \
  template<>                                                                                \
  const std::string sotClassType<sotSigType,sotCoefType>::CLASS_NAME = className;           \
  template<>                                                                                \
  const std::string& sotClassType<sotSigType,sotCoefType>::                                 \
  getClassName( void ) const { return CLASS_NAME; }                                         \
  extern "C" {                                                                              \
    Entity *regFunction##_##sotSigType##_##sotCoefType( const std::string& objname )        \
    {                                                                                       \
      return new sotClassType<sotSigType,sotCoefType>( objname );                           \
    }                                                                                       \
    EntityRegisterer                                                                        \
    regObj##_##sotSigType##_##sotCoefType(sotClassType<sotSigType,sotCoefType>::CLASS_NAME, \
                          &regFunction##_##sotSigType##_##sotCoefType );                    \
  }

using namespace std;
namespace dynamicgraph { 
  namespace sot {
    SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_EULER(IntegratorEuler,Vector,Matrix,
				       "IntegratorEulerVectorMatrix")
    SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_EULER(IntegratorEuler,Vector,double,
				       "IntegratorEulerVectorDouble")
  } // namespace sot
} // namespace dynamicgraph
