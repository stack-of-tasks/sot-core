/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      exception-dynamic.h
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


#ifndef __SOT_DYNAMIC_EXCEPTION_H
#define __SOT_DYNAMIC_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* \class sotExceptionDynamic
 */
class SOT_CORE_EXPORT sotExceptionDynamic 
:public sotExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = sotExceptionAbstract::DYNAMIC

      ,CANT_DESTROY_SIGNAL
      ,JOINT_RANK
      ,DYNAMIC_JRL
      ,JOINT_SIZE
      ,INTEGRATION
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

public:

  sotExceptionDynamic ( const sotExceptionDynamic::ErrorCodeEnum& errcode,
		       const std::string & msg = "" );
  sotExceptionDynamic( const sotExceptionDynamic::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~sotExceptionDynamic( void ){}


};





#endif /* #ifndef __SOT_DYNAMIC_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
