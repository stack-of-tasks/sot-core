/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_DYNAMIC_EXCEPTION_H
#define __SOT_DYNAMIC_EXCEPTION_H

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

/* \class ExceptionDynamic
 */
class SOT_CORE_EXPORT ExceptionDynamic : public ExceptionAbstract

{
 public:
  enum ErrorCodeEnum {
    GENERIC = ExceptionAbstract::DYNAMIC

    ,
    CANT_DESTROY_SIGNAL,
    JOINT_RANK,
    DYNAMIC_JRL,
    JOINT_SIZE,
    INTEGRATION
  };

  static const std::string EXCEPTION_NAME;
  virtual const std::string &getExceptionName(void) const {
    return EXCEPTION_NAME;
  }

 public:
  ExceptionDynamic(const ExceptionDynamic::ErrorCodeEnum &errcode,
                   const std::string &msg = "");
  ExceptionDynamic(const ExceptionDynamic::ErrorCodeEnum &errcode,
                   const std::string &msg, const char *format, ...);
  virtual ~ExceptionDynamic(void) throw() {}
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_DYNAMIC_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
