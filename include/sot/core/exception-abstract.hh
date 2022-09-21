/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_ABSTRACT_EXCEPTION_H
#define __SOT_ABSTRACT_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Classes standards. */
#include <exception>
#include <ostream> /* Classe ostream.    */
#include <string>  /* Classe string.     */

#include "sot/core/api.hh"

// Uncomment this macros to have lines parameter on the throw display
// #define SOT_EXCEPTION_PASSING_PARAM

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/* \class ExceptionAbstract
 */
class SOT_CORE_EXPORT ExceptionAbstract : public std::exception {
 public:
  enum ExceptionEnum {
    ABSTRACT = 0,
    SIGNAL = 100,
    TASK = 200,
    FEATURE = 300,
    FACTORY = 400,
    DYNAMIC = 500,
    TRACES = 600,
    TOOLS = 700,
    PATTERN_GENERATOR = 800
  };

  static const std::string EXCEPTION_NAME;
  virtual const std::string &getExceptionName(void) const {
    return EXCEPTION_NAME;
  }

 protected:
  /** Error code.
   * \sa ErrorCodeEnum */
  int code;

  /**  Error message (can be empty). */
  std::string message;

 private:
  /**  forbid the empty constructor (private). */
  ExceptionAbstract(void);

 public:
  ExceptionAbstract(const int &code, const std::string &msg = "");
  virtual ~ExceptionAbstract(void) throw() {}

  /**  Access to the error code. */
  int getCode(void);

  /** Reference access to the error message (can be empty). */
  const std::string &getStringMessage(void);

  /** Access to the pointer on the array of  \e char related to the error
   * string. Cannot be  \e NULL.
   */
  const char *getMessage(void);
  const char *what() const throw();

  /** Print the error structure. */
  SOT_CORE_EXPORT friend std::ostream &operator<<(std::ostream &os,
                                                  const ExceptionAbstract &err);

#ifdef SOT_EXCEPTION_PASSING_PARAM
 public:
  class Param {
   public:
    static const int BUFFER_SIZE = 80;

    const char *functionPTR;
    char function[BUFFER_SIZE];
    int line;
    const char *filePTR;
    char file[BUFFER_SIZE];
    bool pointersSet, set;

   public:
    Param(const int &_line, const char *_function, const char *_file);
    Param(void) : pointersSet(false), set(false) {}
    Param &initCopy(const Param &p);
  };

 protected:
  mutable Param p;

  template <class Exc>
  friend const Exc &operator+(const ExceptionAbstract::Param &p, const Exc &e) {
    e.p.initCopy(p);
    return e;
  }
  template <class Exc>
  friend Exc &operator+(const ExceptionAbstract::Param &p, Exc &e) {
    e.p.initCopy(p);
    return e;
  }
#endif  //#ifdef SOT_EXCEPTION_PASSING_PARAM
};

#define SOT_RETHROW \
  (const ExceptionAbstract &err) { throw err; }

#ifdef SOT_EXCEPTION_PASSING_PARAM
#define SOT_THROW \
  throw ExceptionAbstract::Param(__LINE__, __FUNCTION__, __FILE__) +
#else  //#ifdef SOT_EXCEPTION_PASSING_PARAM
#define SOT_THROW throw
#endif  //#ifdef SOT_EXCEPTION_PASSING_PARAM

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_ABSTRACT_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
