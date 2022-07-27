/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_EXCEPTION_TASK_H
#define __SOT_EXCEPTION_TASK_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/core/exception-abstract.hh>

#include "sot/core/api.hh"
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/* \class ExceptionTask
 */
class SOT_CORE_EXPORT ExceptionTask : public ExceptionAbstract

{
 public:
  enum ErrorCodeEnum {
    GENERIC = ExceptionAbstract::TASK,
    EMPTY_LIST,
    NON_ADEQUATE_FEATURES,
    MATRIX_SIZE,
    BOUND_TYPE,
    PARSER_MULTI_BOUND
  };

  static const std::string EXCEPTION_NAME;
  virtual const std::string &getExceptionName(void) const {
    return EXCEPTION_NAME;
  }

  ExceptionTask(const ExceptionTask::ErrorCodeEnum &errcode,
                const std::string &msg = "");
  ExceptionTask(const ExceptionTask::ErrorCodeEnum &errcode,
                const std::string &msg, const char *format, ...);
  virtual ~ExceptionTask(void) throw() {}
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_EXCEPTION_TASK_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
