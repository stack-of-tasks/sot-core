/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SIGNAL_EXCEPTION_H
#define __SOT_SIGNAL_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include "sot/core/api.hh"
#include <sot/core/exception-abstract.hh>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/* \class ExceptionSignal
 */
class SOT_CORE_EXPORT ExceptionSignal : public ExceptionAbstract

{
public:
  enum ErrorCodeEnum {
    GENERIC = ExceptionAbstract::SIGNAL

    ,
    READWRITE_LOCK,
    COPY_NOT_INITIALIZED,
    NOT_INITIALIZED,
    PLUG_IMPOSSIBLE,
    SET_IMPOSSIBLE,
    BAD_CAST
  };

  static const std::string EXCEPTION_NAME;
  virtual const std::string &getExceptionName(void) const {
    return EXCEPTION_NAME;
  }

public:
  ExceptionSignal(const ExceptionSignal::ErrorCodeEnum &errcode,
                  const std::string &msg = "");
  ExceptionSignal(const ExceptionSignal::ErrorCodeEnum &errcode,
                  const std::string &msg, const char *format, ...);
  virtual ~ExceptionSignal(void) throw() {}
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_SIGNAL_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
