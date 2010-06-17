/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionSignal.cpp
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

#include <sot-core/sotExceptionSignal.h>
#include <stdarg.h>
#include <cstdio>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string sotExceptionSignal::EXCEPTION_NAME = "Signal";

sotExceptionSignal::
sotExceptionSignal ( const sotExceptionSignal::ErrorCodeEnum& errcode,
		     const std::string & msg )
  :sotExceptionAbstract(errcode,msg)
{
}

sotExceptionSignal::
sotExceptionSignal ( const sotExceptionSignal::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... )
  :sotExceptionAbstract(errcode,msg)
{
  va_list args;
  va_start(args,format);

  const unsigned int SIZE = 256;
  char  buffer[SIZE];
  vsnprintf(buffer,SIZE,format,args);

  message += buffer;

  va_end(args);
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
