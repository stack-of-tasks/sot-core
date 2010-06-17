/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionFeature.h
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


#include <sot-core/sotExceptionAbstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* \class sotExceptionFeature
 */
class SOT_CORE_EXPORT sotExceptionFeature 
:public sotExceptionAbstract

{
public:

  enum ErrorCodeEnum
    {
      GENERIC = sotExceptionAbstract::FEATURE
      ,BAD_INIT
      ,UNCOMPATIBLE_SIZE
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return sotExceptionFeature::EXCEPTION_NAME; }

  sotExceptionFeature ( const sotExceptionFeature::ErrorCodeEnum& errcode,
		     const std::string & msg = "" );

  sotExceptionFeature ( const sotExceptionFeature::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );

  virtual ~sotExceptionFeature( void ){}
};





#endif /* #ifndef __SOT_EXCEPTION_FEATURE_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
