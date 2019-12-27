/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <cstdio>
#include <sot/core/exception-feature.hh>
#include <stdarg.h>

using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionFeature::EXCEPTION_NAME = "Feature";

ExceptionFeature::ExceptionFeature(
    const ExceptionFeature::ErrorCodeEnum &errcode, const std::string &msg)
    : ExceptionAbstract(errcode, msg) {}

ExceptionFeature::ExceptionFeature(
    const ExceptionFeature::ErrorCodeEnum &errcode, const std::string &msg,
    const char *format, ...)
    : ExceptionAbstract(errcode, msg) {
  va_list args;
  va_start(args, format);

  const unsigned int SIZE = 256;
  char buffer[SIZE];
  vsnprintf(buffer, SIZE, format, args);

  message = buffer;

  va_end(args);
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
