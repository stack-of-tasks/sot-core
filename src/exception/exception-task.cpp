/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <stdarg.h>

#include <cstdio>
#include <sot/core/exception-task.hh>

using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionTask::EXCEPTION_NAME = "Task";

ExceptionTask::ExceptionTask(const ExceptionTask::ErrorCodeEnum &errcode,
                             const std::string &msg)
    : ExceptionAbstract(errcode, msg) {}

ExceptionTask::ExceptionTask(const ExceptionTask::ErrorCodeEnum &errcode,
                             const std::string &msg, const char *format, ...)
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
