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

#include <sot/core/fir-filter.hh>
#include <sot/core/factory.hh>

//using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotClassType,sotSigType,sotCoefType,id,className) \
  template<>								\
  std::string sotClassType<sotSigType,sotCoefType>::			\
  getTypeName( void ) { return #sotSigType; }				\
									\
  template<>								\
  const std::string sotClassType<sotSigType,sotCoefType>::CLASS_NAME	\
  = std::string(className)+"<"+#sotSigType+","+#sotCoefType+">";	\
									\
  template<>								\
  const std::string& sotClassType<sotSigType,sotCoefType>::		\
  getClassName( void ) const { return CLASS_NAME; }				\
extern "C" {								\
  Entity *regFunction##_##id ( const std::string& objname )		\
  {									\
    return new sotClassType<sotSigType,sotCoefType>( objname );		\
  }									\
  EntityRegisterer reg##_##id					\
  ( std::string(className)+"<"+#sotSigType+","+#sotCoefType+">",	\
    &regFunction##_##id );					\
}

using namespace ml;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(FIRFilter,double,double,double_double,"FIRFilter")
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(FIRFilter,Vector,double,vec_double,"FIRFilter")
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(FIRFilter,Vector,Matrix,vec_mat,"FIRFilter")

template<>
void FIRFilter<Vector, double>::reset_signal( Vector& res, const Vector& sample )
{
  res.resize(sample.size());
  res.fill(0);
}

template<>
void FIRFilter<Vector, Matrix>::reset_signal( Vector& res, const Vector& sample )
{
  res.resize(sample.size());
  res.fill(0);
}

#include <sot/core/fir-filter-impl.hh>

#ifdef WIN32
#define DEFINE_SPECIFICATION(sotClassType,sotSigType,sotCoefType) \
  sotClassType##sotSigType##sotCoefType::sotClassType##sotSigType##sotCoefType(const std::string& name):			\
  sotClassType<sotSigType,sotCoefType> (name) {}; \

typedef double Double;
 DEFINE_SPECIFICATION(FIRFilter,Double,Double)
 DEFINE_SPECIFICATION(FIRFilter,Vector,Double)
 DEFINE_SPECIFICATION(FIRFilter,Vector,Matrix)
#endif //WIN32
