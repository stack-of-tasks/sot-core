/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <cstdio>
#include <sot/core/debug.hh>
#include <sot/core/exception-factory.hh>
#include <stdarg.h>

using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionFactory::EXCEPTION_NAME = "Factory";

ExceptionFactory::ExceptionFactory(
    const ExceptionFactory::ErrorCodeEnum &errcode, const std::string &msg)
    : ExceptionAbstract(errcode, msg) {
  sotDEBUGF(15, "Created with message <%s>.", msg.c_str());
  sotDEBUG(1) << "Created with message <%s>." << msg << std::endl;
}

ExceptionFactory::ExceptionFactory(
    const ExceptionFactory::ErrorCodeEnum &errcode, const std::string &msg,
    const char *format, ...)
    : ExceptionAbstract(errcode, msg) {
  va_list args;
  va_start(args, format);

  const unsigned int SIZE = 256;
  char buffer[SIZE];
  vsnprintf(buffer, SIZE, format, args);

  sotDEBUG(15) << "Created "
               << " with message <" << msg << "> and buffer <" << buffer
               << ">. " << std::endl;

  message += buffer;

  va_end(args);

  sotDEBUG(1) << "Throw exception " << EXCEPTION_NAME << "[#" << errcode
              << "]: "
              << "<" << message << ">." << std::endl;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
