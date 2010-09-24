/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      exception-tools.h
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


#ifndef __SOT_TOOLS_EXCEPTION_H
#define __SOT_TOOLS_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/* \class ExceptionTools
 */
class SOT_CORE_EXPORT ExceptionTools 
:public ExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::TOOLS

      ,CORBA
      ,KALMAN_SIZE
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

public:

  ExceptionTools ( const ExceptionTools::ErrorCodeEnum& errcode,
		       const std::string & msg = "" );
  ExceptionTools( const ExceptionTools::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~ExceptionTools( void ) throw() {}


};

} // namespace sot



#endif /* #ifndef __SOT_TOOLS_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
