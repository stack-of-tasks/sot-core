/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotIntegratorEuler.cpp
 * Project:   SOT
 * Author:    Paul Evrard and Nicolas Mansard
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

#include <sot-core/sotIntegratorEuler.h>

#include <sot-core/sotFactory.h>

#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotClassType,sotSigType,sotCoefType,className)   \
  template<>                                                                                \
  std::string sotClassType<sotSigType,sotCoefType>::                                        \
  getTypeName( void ) { return #sotSigType; }                                               \
  template<>                                                                                \
  const std::string sotClassType<sotSigType,sotCoefType>::CLASS_NAME                        \
     = std::string(className)+"<"+#sotSigType+","+#sotCoefType+">";                         \
  template<>                                                                                \
  const std::string& sotClassType<sotSigType,sotCoefType>::                                 \
  getClassName( void ) const { return CLASS_NAME; }                                               \
  extern "C" {                                                                              \
    Entity *regFunction##_##sotSigType( const std::string& objname )                     \
    {                                                                                       \
      return new sotClassType<sotSigType,sotCoefType>( objname );                           \
    }                                                                                       \
    EntityRegisterer                                                                     \
    regObj##_##sotSigType(std::string(className)+"<"+#sotSigType+","+#sotCoefType+">",      \
                          &regFunction##_##sotSigType );                                    \
  }

using namespace ml;
using namespace std;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotIntegratorEuler,Vector,Matrix,"integratorEuler");
