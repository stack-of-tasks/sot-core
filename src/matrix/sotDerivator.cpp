/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotDerivator.h
 * Project:   SOT
 * Author:    Nicolas Mansard
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

#include <sot-core/sotDerivator.h>
#include <sot-core/sotFactory.h>



#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotClassType,sotType,className)              \
  template<>                                                                            \
  const double sotDerivator<sotType>::TIMESTEP_DEFAULT = 1.;                            \
  template<>                                                                            \
  std::string sotClassType<sotType>::                                                   \
  getTypeName( void ) { return #sotType; }                                              \
  template<>                                                                            \
  const std::string sotClassType<sotType>::CLASS_NAME                                   \
     = std::string(className)+"<"+#sotType+">";                                         \
  extern "C" {                                                                          \
    Entity *regFunction##_##sotType##_##sotClassType( const std::string& objname )                    \
    {                                                                                   \
      return new sotClassType<sotType>( objname );                                      \
    }                                                                                   \
  EntityRegisterer regObj##_##sotType##_##sotClassType( std::string(className)+"<"+#sotType+">",      \
					  &regFunction##_##sotType##_##sotClassType );                   \
  }

using namespace ml;
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,double,"Derivator");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,Vector,"Derivator");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,Matrix,"Derivator");
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,sotVectorQuaternion,"Derivator");



//SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,double,"T");


/*
#define SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotClassType,sotType,className,typeName,regFunction,regObj) \
  template<>                                                \
  std::string sotClassType<sotType>::                       \
  getTypeName( void ) { return typeName; }                  \
  template<>                                                \
  const std::string sotClassType<sotType>::CLASS_NAME = std::string(className)+typeName; \
  extern "C" {                                              \
    Entity *regFunction( const std::string& objname )    \
    {                                                       \
      return new sotClassType<sotType>( objname );          \
    }                                                       \
  EntityRegisterer regObj( std::string(className)+typeName,&regFunction );   \
  }


SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,double,"derivator","Double",rfd,rod);
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,ml::Vector,"derivator","Vector",rfv,rov);
SOT_FACTORY_TEMPLATE_ENTITY_PLUGIN(sotDerivator,ml::Matrix,"derivator","Matrix",rfm,rom);

*/

