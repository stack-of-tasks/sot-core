/*
 * Copyright 2018,
 * Mirabel Joseph
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_CORE_VARIADICOP_HH
#define SOT_CORE_VARIADICOP_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/flags.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/pool.hh>

/* STD */
#include <boost/function.hpp>
#include <string>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

template <typename Tin, typename Tout, typename Time>
class VariadicAbstract : public Entity {
 public: /* --- CONSTRUCTION --- */
  static std::string getTypeInName(void);
  static std::string getTypeOutName(void);

  VariadicAbstract(const std::string &name, const std::string &className)
      : Entity(name),
        SOUT(className + "(" + name + ")::output(" + getTypeOutName() +
             ")::sout"),
        baseSigname(className + "(" + name + ")::input(" + getTypeInName() +
                    ")::") {
    signalRegistration(SOUT);
  }

  virtual ~VariadicAbstract(void) {
    for (std::size_t i = 0; i < signalsIN.size(); ++i) {
      _removeSignal(i);
    }
  };

 public: /* --- SIGNAL --- */
  typedef SignalPtr<Tin, int> signal_t;
  SignalTimeDependent<Tout, int> SOUT;

  std::size_t addSignal() {
    std::ostringstream oss;
    oss << "sin" << signalsIN.size();
    return addSignal(oss.str());
  }

  std::size_t addSignal(const std::string &name) {
    signal_t *sig = new signal_t(NULL, baseSigname + name);
    try {
      _declareSignal(sig);
      signalsIN.push_back(sig);
      // names.push_back (name);
      return signalsIN.size() - 1;
    } catch (const ExceptionAbstract &) {
      delete sig;
      throw;
    }
  }

  void removeSignal() {
    assert(signalsIN.size() > 0);
    _removeSignal(signalsIN.size() - 1);
    // names.pop_back();
    signalsIN.pop_back();
  }

  void setSignalNumber(const int &n) {
    assert(n >= 0);
    const std::size_t oldSize = signalsIN.size();
    for (std::size_t i = n; i < oldSize; ++i) _removeSignal(i);
    signalsIN.resize(n, NULL);
    // names.resize(n);

    for (std::size_t i = oldSize; i < (std::size_t)n; ++i) {
      assert(signalsIN[i] == NULL);
      std::ostringstream oss;
      oss << baseSigname << "sin" << i;
      // names[i] = oss.str();
      // signal_t* s = new signal_t (NULL,names[i]);
      signal_t *s = new signal_t(NULL, oss.str());
      signalsIN[i] = s;
      _declareSignal(s);
    }
    updateSignalNumber(n);
  }

  int getSignalNumber() const { return (int)signalsIN.size(); }

  signal_t *getSignalIn(int i) {
    if (i < 0 || i >= (int)signalsIN.size())
      throw std::out_of_range("Wrong signal index");
    return signalsIN[i];
  }

 protected:
  std::vector<signal_t *> signalsIN;
  // Use signal->shortName instead
  // std::vector< std::string > names;

  virtual void updateSignalNumber(int n) { (void)n; };

 private:
  void _removeSignal(const std::size_t i) {
    // signalDeregistration(names[i]);
    signalDeregistration(signalsIN[i]->shortName());
    SOUT.removeDependency(*signalsIN[i]);
    delete signalsIN[i];
  }
  void _declareSignal(signal_t *s) {
    signalRegistration(*s);
    SOUT.addDependency(*s);
  }
  const std::string baseSigname;
};

template <typename Operator>
class VariadicOp : public VariadicAbstract<typename Operator::Tin,
                                           typename Operator::Tout, int> {
  typedef typename Operator::Tin Tin;
  typedef typename Operator::Tout Tout;
  typedef VariadicOp<Operator> Self;

 public: /* --- CONSTRUCTION --- */
  Operator op;

  typedef VariadicAbstract<Tin, Tout, int> Base;

  // static std::string getTypeInName ( void ) { return Operator::nameTypeIn ();
  // } static std::string getTypeOutName( void ) { return
  // Operator::nameTypeOut(); }
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName() const { return CLASS_NAME; }
  std::string getDocString() const { return op.getDocString(); }

  VariadicOp(const std::string &name) : Base(name, CLASS_NAME) {
    this->SOUT.setFunction(boost::bind(&Self::computeOperation, this, _1, _2));
    op.initialize(this, this->commandMap);
  }

  virtual ~VariadicOp(void){};

 protected:
  Tout &computeOperation(Tout &res, int time) {
    std::vector<const Tin *> in(this->signalsIN.size());
    for (std::size_t i = 0; i < this->signalsIN.size(); ++i) {
      const Tin &x = this->signalsIN[i]->access(time);
      in[i] = &x;
    }
    op(in, res);
    return res;
  }

  inline void updateSignalNumber(int n) { op.updateSignalNumber(n); }
};
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef SOT_CORE_VARIADICOP_HH
