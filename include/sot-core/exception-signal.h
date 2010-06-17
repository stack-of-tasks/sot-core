/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      exception-signal.h
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


#ifndef __SOT_SIGNAL_EXCEPTION_H
#define __SOT_SIGNAL_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* \class sotExceptionSignal
 */
class SOT_CORE_EXPORT sotExceptionSignal 
:public sotExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = sotExceptionAbstract::SIGNAL

      ,READWRITE_LOCK
      ,COPY_NOT_INITIALIZED
      ,NOT_INITIALIZED
      ,PLUG_IMPOSSIBLE
      ,SET_IMPOSSIBLE
      ,BAD_CAST
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

public:

  sotExceptionSignal ( const sotExceptionSignal::ErrorCodeEnum& errcode,
		       const std::string & msg = "" );
  sotExceptionSignal( const sotExceptionSignal::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~sotExceptionSignal( void ){}


};





#endif /* #ifndef __SOT_SIGNAL_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
