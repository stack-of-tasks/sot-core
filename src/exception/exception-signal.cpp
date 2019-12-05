/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <cstdio>
#include <sot/core/exception-signal.hh>
#include <stdarg.h>

using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionSignal::EXCEPTION_NAME = "Signal";

ExceptionSignal::ExceptionSignal(const ExceptionSignal::ErrorCodeEnum &errcode,
                                 const std::string &msg)
    : ExceptionAbstract(errcode, msg) {}

ExceptionSignal::ExceptionSignal(const ExceptionSignal::ErrorCodeEnum &errcode,
                                 const std::string &msg, const char *format,
                                 ...)
    : ExceptionAbstract(errcode, msg) {
  va_list args;
  va_start(args, format);

  const unsigned int SIZE = 256;
  char buffer[SIZE];
  vsnprintf(buffer, SIZE, format, args);

  message += buffer;

  va_end(args);
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
