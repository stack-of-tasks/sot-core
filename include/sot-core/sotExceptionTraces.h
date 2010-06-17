/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionTraces.h
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


#ifndef __SOT_TRACES_EXCEPTION_H
#define __SOT_TRACES_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/sotExceptionAbstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* \class sotExceptionTraces
 */
class SOT_CORE_EXPORT sotExceptionTraces 
:public sotExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = sotExceptionAbstract::TRACES

      ,NOT_OPEN
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

public:

  sotExceptionTraces ( const sotExceptionTraces::ErrorCodeEnum& errcode,
		       const std::string & msg = "" );
  sotExceptionTraces( const sotExceptionTraces::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~sotExceptionTraces( void ){}


};





#endif /* #ifndef __SOT_TRACES_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
