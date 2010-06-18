/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      exception-factory.h
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


#ifndef __SOT_EXCEPTION_FACTORY_H
#define __SOT_EXCEPTION_FACTORY_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/* \class sotExceptionFactory
 */
class SOT_CORE_EXPORT sotExceptionFactory 
:public sotExceptionAbstract

{
public:

  enum ErrorCodeEnum
    {
      GENERIC = sotExceptionAbstract::FACTORY
      ,UNREFERED_OBJECT
      ,UNREFERED_SIGNAL
      ,UNREFERED_FUNCTION
      ,DYNAMIC_LOADING
      ,SIGNAL_CONFLICT
      ,FUNCTION_CONFLICT
      ,OBJECT_CONFLICT
      ,SYNTAX_ERROR    // j' aime bien FATAL_ERROR aussi faut que je la case qq part...
      ,READ_FILE
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void )const{ return sotExceptionFactory::EXCEPTION_NAME; }

  sotExceptionFactory ( const sotExceptionFactory::ErrorCodeEnum& errcode,
			const std::string & msg = "" );
  sotExceptionFactory ( const sotExceptionFactory::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~sotExceptionFactory( void ){}

};


} // namespace sot


#endif /* #ifndef __SOT_EXCEPTION_FACTORY_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
