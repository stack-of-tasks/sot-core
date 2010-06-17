/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2008
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFIRFilter.cpp
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

#include <sot-core/sotFIRFilter.h>

#include <sot-core/factory.h>

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
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotFIRFilter,double,double,double_double,"FIRFilter");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotFIRFilter,Vector,double,vec_double,"FIRFilter");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotFIRFilter,Vector,Matrix,vec_mat,"FIRFilter");

template<>
void sotFIRFilter<Vector, double>::reset_signal( Vector& res, const Vector& sample )
{
  res.resize(sample.size());
  res.fill(0);
}

template<>
void sotFIRFilter<Vector, Matrix>::reset_signal( Vector& res, const Vector& sample )
{
  res.resize(sample.size());
  res.fill(0);
}

