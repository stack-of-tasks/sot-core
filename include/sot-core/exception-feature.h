/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      exception-feature.h
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


#ifndef __SOT_EXCEPTION_FEATURE_H
#define __SOT_EXCEPTION_FEATURE_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/* \class ExceptionFeature
 */
class SOT_CORE_EXPORT ExceptionFeature 
:public ExceptionAbstract

{
public:

  enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::FEATURE
      ,BAD_INIT
      ,UNCOMPATIBLE_SIZE
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return ExceptionFeature::EXCEPTION_NAME; }

  ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
		     const std::string & msg = "" );

  ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );

  virtual ~ExceptionFeature( void ){}
};


} // namespace sot


#endif /* #ifndef __SOT_EXCEPTION_FEATURE_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
