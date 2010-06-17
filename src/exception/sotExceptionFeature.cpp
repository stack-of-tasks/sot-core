/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionFeature.cpp
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

#include <sot-core/sotExceptionFeature.h>
#include <stdarg.h>
#include <cstdio>


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string sotExceptionFeature::EXCEPTION_NAME = "Feature";

sotExceptionFeature::
sotExceptionFeature ( const sotExceptionFeature::ErrorCodeEnum& errcode,
		      const std::string & msg )
  :sotExceptionAbstract(errcode,msg)
{
}

sotExceptionFeature::
sotExceptionFeature ( const sotExceptionFeature::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... )
  :sotExceptionAbstract(errcode,msg)
{
  va_list args;
  va_start(args,format);

  const unsigned int SIZE = 256;
  char  buffer[SIZE];
  vsnprintf(buffer,SIZE,format,args);

  message = buffer;

  va_end(args);
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
