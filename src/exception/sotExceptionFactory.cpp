/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionFactory.cpp
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

#include <sot-core/sotExceptionFactory.h>
#include <sot-core/sotDebug.h>
#include <stdarg.h>
#include <cstdio>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string sotExceptionFactory::EXCEPTION_NAME = "Factory";

sotExceptionFactory::
sotExceptionFactory ( const sotExceptionFactory::ErrorCodeEnum& errcode,
		      const std::string & msg )
  :sotExceptionAbstract(errcode,msg)
{
  sotDEBUGF( 15,"Created with message <%s>.",msg.c_str());
  sotDEBUG( 1) <<"Created with message <%s>."<<msg<<std::endl;
}

sotExceptionFactory::
sotExceptionFactory ( const sotExceptionFactory::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... )
  :sotExceptionAbstract(errcode,msg)
{
  va_list args;
  va_start(args,format);

  const unsigned int SIZE = 256;
  char  buffer[SIZE];
  vsnprintf(buffer,SIZE,format,args);

  sotDEBUG(15) <<"Created "<<" with message <"
	       <<msg<<"> and buffer <"<<buffer<<">. "<<std::endl;

  message += buffer;

  va_end(args);

  sotDEBUG(1) << "Throw exception " << EXCEPTION_NAME << "[#" << errcode<<"]: " 
	      <<"<"<< message << ">."<<std::endl;

}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
