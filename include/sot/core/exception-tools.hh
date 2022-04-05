/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_TOOLS_EXCEPTION_H
#define __SOT_TOOLS_EXCEPTION_H

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

/* \class ExceptionTools
 */
class SOT_CORE_EXPORT ExceptionTools : public ExceptionAbstract

{
 public:
  enum ErrorCodeEnum {
    GENERIC = ExceptionAbstract::TOOLS

    ,
    CORBA,
    KALMAN_SIZE,
    PARAMETER_SERVER
  };

  static const std::string EXCEPTION_NAME;
  virtual const std::string &getExceptionName() const { return EXCEPTION_NAME; }

 public:
  ExceptionTools(const ExceptionTools::ErrorCodeEnum &errcode,
                 const std::string &msg = "");
  ExceptionTools(const ExceptionTools::ErrorCodeEnum &errcode,
                 const std::string &msg, const char *format, ...);
  virtual ~ExceptionTools(void) throw() {}
};

}  // namespace sot
}  // namespace dynamicgraph

#endif /* #ifndef __SOT_TOOLS_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
