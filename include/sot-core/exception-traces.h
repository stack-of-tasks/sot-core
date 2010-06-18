/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      exception-traces.h
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


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/* \class sotExceptionTraces
 */
class SOT_CORE_EXPORT sotExceptionTraces 
:public ExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::TRACES

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

} // namespace sot



#endif /* #ifndef __SOT_TRACES_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
