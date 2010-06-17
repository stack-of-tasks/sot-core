/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionTools.h
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


#include <sot-core/sotExceptionAbstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* \class sotExceptionTools
 */
class SOT_CORE_EXPORT sotExceptionTools 
:public sotExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = sotExceptionAbstract::TOOLS

      ,CORBA
      ,KALMAN_SIZE
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

public:

  sotExceptionTools ( const sotExceptionTools::ErrorCodeEnum& errcode,
		       const std::string & msg = "" );
  sotExceptionTools( const sotExceptionTools::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~sotExceptionTools( void ){}


};





#endif /* #ifndef __SOT_TOOLS_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
