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

#include <sot/core/derivator.hh>
#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace ml;
using namespace dynamicgraph;

// Unrolled Code:
// template<>
// const double Derivator<double>::TIMESTEP_DEFAULT = 1.;

// template<>
// std::string Derivator<double>::
// getTypeName( void ) { return #double; }

// template<>
// const std::string Derivator<double>::CLASS_NAME = std::string("Derivator_of_double");
// extern "C" {
//   Entity *regFunctiondoubleDerivator( const std::string& objname )
//   {
//     return new Derivator<double>( objname );
//   }
//   EntityRegisterer regObjdoubleDerivator
//   ( std::string("Derivator_of_double"),
//     &regFunctiondoubleDerivator );
// }


#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotClassType,sotType,className)              \
  template<>                                                                            \
  const double Derivator<sotType>::TIMESTEP_DEFAULT = 1.;                               \
  template<>                                                                            \
  std::string sotClassType<sotType>::                                                   \
  getTypeName( void ) { return #sotType; }                                              \
  template<>                                                                            \
  const std::string sotClassType<sotType>::CLASS_NAME                                   \
    = std::string(className)+"_of_"+#sotType;                                           \
  extern "C" {                                                                          \
    Entity *regFunction##_##sotType##_##sotClassType( const std::string& objname )      \
    {                                                                                   \
      return new sotClassType<sotType>( objname );                                      \
    }                                                                                   \
  EntityRegisterer regObj##_##sotType##_##sotClassType                                  \
     ( std::string(className)+"_of_"+#sotType,				                \
       &regFunction##_##sotType##_##sotClassType );                                     \
  }

namespace dynamicgraph {
  namespace sot {
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(Derivator,double,"Derivator")
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(Derivator,Vector,"Derivator")
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(Derivator,Matrix,"Derivator")
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(Derivator,VectorQuaternion,"Derivator")
  } // namespace sot
} // namespace dynamicgraph

#include <sot/core/derivator-impl.hh>

#ifdef WIN32
# define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_WIN32(sotClassType,sotType,className)        \
  sotClassType##sotType##::sotClassType##sotType##(const std::string& name):			\
  sotClassType<sotType> (name) {};                                                      \

  typedef double Double;
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_WIN32(Derivator,Double,"Derivator")
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_WIN32(Derivator,Vector,"Derivator")
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_WIN32(Derivator,Matrix,"Derivator")
  SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN_WIN32(Derivator,VectorQuaternion,"Derivator")
#endif
