/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FIRFILTER_HH__
#define __SOT_FIRFILTER_HH__

#include <cassert>

#include <algorithm>
#include <iterator>
#include <vector>

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/entity.h>

namespace dynamicgraph {
namespace sot {

namespace detail {
// GRMBL boost-sandox::circular_buffer smells... Why keep using 1.33?!
// As a workaround, only the part of circular_buffer's interface used
// here is implemented.
// Ugly, fatty piece of code.
template <class T> class circular_buffer {
public:
  circular_buffer() : buf(1), start(0), numel(0) {}
  void push_front(const T &data) {
    if (start) {
      --start;
    } else {
      start = buf.size() - 1;
    }
    buf[start] = data;
    if (numel < buf.size()) {
      ++numel;
    }
  }
  void reset_capacity(size_t n) {
    buf.resize(n);
    start = 0;
    numel = 0;
  }
  void reset_capacity(size_t n, const T &el) {
    buf.clear();
    buf.resize(n, el);
    start = 0;
    numel = 0;
  }
  T &operator[](size_t i) {
    assert((i < numel) && "Youre accessing an empty buffer");
    size_t index = (start + i) % buf.size();
    return buf[index];
  }
  size_t size() const { return numel; }

private:
  std::vector<T> buf;
  size_t start;
  size_t numel;
}; // class circular_buffer
} // namespace detail

template <class sigT, class coefT> class FIRFilter;

namespace command {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

template <class sigT, class coefT> class SetElement : public Command {
public:
  SetElement(FIRFilter<sigT, coefT> &entity, const std::string &docstring);
  Value doExecute();
}; // class SetElement

template <class sigT, class coefT> class GetElement : public Command {
public:
  GetElement(FIRFilter<sigT, coefT> &entity, const std::string &docstring);
  Value doExecute();
}; // class SetElement
} // namespace command

using ::dynamicgraph::command::Getter;
using ::dynamicgraph::command::Setter;

template <class sigT, class coefT> class FIRFilter : public Entity {
public:
  virtual const std::string &getClassName() const {
    return Entity::getClassName();
  }
  static std::string getTypeName(void) { return "Unknown"; }
  static const std::string CLASS_NAME;

  std::string getDocString() const {
    return "Finite impulse response filter\n"
           "\n"
           "  Provide the following sum in output signal:\n"
           "             N                   \n"
           "             __                  \n"
           "     y (n) = \\   c  s (n-i)      \n"
           "             /_   i              \n"
           "             i=0                 \n"
           "                                 \n"
           "  where\n"
           "    -  c_i are coefficients stored in an array\n"
           "    -  N is the size of the array\n"
           "    -  s is the input signal.\n";
  }

public:
  FIRFilter(const std::string &name)
      : Entity(name), SIN(NULL, "sotFIRFilter(" + name + ")::input(T)::sin"),
        SOUT(boost::bind(&FIRFilter::compute, this, _1, _2), SIN,
             "sotFIRFilter(" + name + ")::output(T)::sout") {
    signalRegistration(SIN << SOUT);
    std::string docstring = "  Set element at rank in array of coefficients\n"
                            "\n"
                            "    Input:\n"
                            "      - positive int: rank\n"
                            "      - element\n";
    addCommand("setElement",
               new command::SetElement<sigT, coefT>(*this, docstring));
    docstring = "  Get element at rank in array of coefficients\n"
                "\n"
                "    Input:\n"
                "      - positive int: rank\n"
                "    Return:\n"
                "      - element\n";
    addCommand("getElement",
               new command::GetElement<sigT, coefT>(*this, docstring));
    docstring = "  Set number of coefficients\n"
                "\n"
                "    Input:\n"
                "      - positive int: size\n";
    addCommand("setSize", new Setter<FIRFilter, unsigned>(
                              *this, &FIRFilter::resizeBuffer, docstring));

    docstring = "  Get Number of coefficients\n"
                "\n"
                "    Return:\n"
                "      - positive int: size\n";
    addCommand("getSize", new Getter<FIRFilter, unsigned>(
                              *this, &FIRFilter::getBufferSize, docstring));
  }

  virtual ~FIRFilter() {}

  virtual sigT &compute(sigT &res, int time) {
    const sigT &in = SIN.access(time);
    reset_signal(res, in);
    data.push_front(in);

    size_t SIZE = std::min(data.size(), coefs.size());
    for (size_t i = 0; i < SIZE; ++i) {
      res += coefs[i] * data[i];
    }

    return res;
  }

  void resizeBuffer(const unsigned int &size) {
    size_t s = static_cast<size_t>(size);
    data.reset_capacity(s);
    coefs.resize(s);
  }

  unsigned int getBufferSize() const {
    return static_cast<unsigned int>(coefs.size());
  }

  void setElement(const unsigned int &rank, const coefT &coef) {
    coefs[rank] = coef;
  }

  coefT getElement(const unsigned int &rank) const { return coefs[rank]; }

  static void reset_signal(sigT & /*res*/, const sigT & /*sample*/) {}

public:
  SignalPtr<sigT, int> SIN;
  SignalTimeDependent<sigT, int> SOUT;

private:
  std::vector<coefT> coefs;
  detail::circular_buffer<sigT> data;
}; // class FIRFilter

namespace command {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;
using ::dynamicgraph::command::ValueHelper;

template <class sigT, class coefT>
SetElement<sigT, coefT>::SetElement(FIRFilter<sigT, coefT> &entity,
                                    const std::string &docstring)
    : Command(
          entity,
          boost::assign::list_of(Value::UNSIGNED)(ValueHelper<coefT>::TypeID),
          docstring) {}

template <class sigT, class coefT> Value SetElement<sigT, coefT>::doExecute() {
  FIRFilter<sigT, coefT> &entity =
      static_cast<FIRFilter<sigT, coefT> &>(owner());
  std::vector<Value> values = getParameterValues();
  unsigned int rank = values[0].value();
  coefT coef = values[1].value();
  entity.setElement(rank, coef);
  return Value();
}

template <class sigT, class coefT>
GetElement<sigT, coefT>::GetElement(FIRFilter<sigT, coefT> &entity,
                                    const std::string &docstring)
    : Command(entity, boost::assign::list_of(Value::UNSIGNED), docstring) {}

template <class sigT, class coefT> Value GetElement<sigT, coefT>::doExecute() {
  FIRFilter<sigT, coefT> &entity =
      static_cast<FIRFilter<sigT, coefT> &>(owner());
  std::vector<Value> values = getParameterValues();
  unsigned int rank = values[0].value();
  return Value(entity.getElement(rank));
}
} // namespace command

} // namespace sot
} // namespace dynamicgraph

#endif
