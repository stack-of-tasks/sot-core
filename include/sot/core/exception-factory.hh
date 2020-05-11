/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_EXCEPTION_FACTORY_H
#define __SOT_EXCEPTION_FACTORY_H

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

/* \class ExceptionFactory
 */
class SOT_CORE_EXPORT ExceptionFactory : public ExceptionAbstract

{
public:
  enum ErrorCodeEnum {
    GENERIC = ExceptionAbstract::FACTORY,
    UNREFERED_OBJECT,
    UNREFERED_SIGNAL,
    UNREFERED_FUNCTION,
    DYNAMIC_LOADING,
    SIGNAL_CONFLICT,
    FUNCTION_CONFLICT,
    OBJECT_CONFLICT,
    SYNTAX_ERROR // j' aime bien FATAL_ERROR aussi faut que je la case qq
                 // part...
    ,
    READ_FILE
  };

  static const std::string EXCEPTION_NAME;
  virtual const std::string &getExceptionName(void) const {
    return ExceptionFactory::EXCEPTION_NAME;
  }

  ExceptionFactory(const ExceptionFactory::ErrorCodeEnum &errcode,
                   const std::string &msg = "");
  ExceptionFactory(const ExceptionFactory::ErrorCodeEnum &errcode,
                   const std::string &msg, const char *format, ...);
  virtual ~ExceptionFactory(void) throw() {}
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_EXCEPTION_FACTORY_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
