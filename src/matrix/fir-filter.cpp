/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2008
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      FIRFilter.cpp
 * Project:   SOT
 * Author:    Paul Evrard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot-core/fir-filter.h>
#include <sot-core/factory.h>

using namespace sot;


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
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(FIRFilter,double,double,double_double,"FIRFilter");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(FIRFilter,Vector,double,vec_double,"FIRFilter");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(FIRFilter,Vector,Matrix,vec_mat,"FIRFilter");

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

