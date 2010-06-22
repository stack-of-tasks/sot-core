/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      ExceptionFeature.cpp
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

#include <sot-core/exception-feature.h>
#include <stdarg.h>
#include <cstdio>

using namespace sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionFeature::EXCEPTION_NAME = "Feature";

ExceptionFeature::
ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
		      const std::string & msg )
  :ExceptionAbstract(errcode,msg)
{
}

ExceptionFeature::
ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... )
  :ExceptionAbstract(errcode,msg)
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
