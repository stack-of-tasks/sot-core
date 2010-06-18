/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionTools.cpp
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

#include <sot-core/exception-tools.h>
#include <stdarg.h>
#include <cstdio>

using namespace sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string sotExceptionTools::EXCEPTION_NAME = "Tools";

sotExceptionTools::
sotExceptionTools ( const sotExceptionTools::ErrorCodeEnum& errcode,
		     const std::string & msg )
  :sotExceptionAbstract(errcode,msg)
{
}

sotExceptionTools::
sotExceptionTools ( const sotExceptionTools::ErrorCodeEnum& errcode,
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
