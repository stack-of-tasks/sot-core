/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_CORE_BINARYOP_HH
#define SOT_CORE_BINARYOP_HH

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

template <typename Operator>
class BinaryOp : public Entity {
  Operator op;
  typedef typename Operator::Tin1 Tin1;
  typedef typename Operator::Tin2 Tin2;
  typedef typename Operator::Tout Tout;
  typedef BinaryOp<Operator> Self;

 public: /* --- CONSTRUCTION --- */
  static std::string getTypeIn1Name(void) { return Operator::nameTypeIn1(); }
  static std::string getTypeIn2Name(void) { return Operator::nameTypeIn2(); }
  static std::string getTypeOutName(void) { return Operator::nameTypeOut(); }
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName() const { return CLASS_NAME; }
  std::string getDocString() const { return op.getDocString(); }

  BinaryOp(const std::string &name)
      : Entity(name),
        SIN1(NULL, BinaryOp::CLASS_NAME + "(" + name + ")::input(" +
                       getTypeIn1Name() + ")::sin1"),
        SIN2(NULL, CLASS_NAME + "(" + name + ")::input(" + getTypeIn2Name() +
                       ")::sin2"),
        SOUT(boost::bind(&Self::computeOperation, this, _1, _2), SIN1 << SIN2,
             CLASS_NAME + "(" + name + ")::output(" + getTypeOutName() +
                 ")::sout") {
    signalRegistration(SIN1 << SIN2 << SOUT);
    op.addSpecificCommands(*this, commandMap);
  }

  virtual ~BinaryOp(void){};

 public: /* --- SIGNAL --- */
  SignalPtr<Tin1, int> SIN1;
  SignalPtr<Tin2, int> SIN2;
  SignalTimeDependent<Tout, int> SOUT;

 protected:
  Tout &computeOperation(Tout &res, int time) {
    const Tin1 &x1 = SIN1(time);
    const Tin2 &x2 = SIN2(time);
    op(x1, x2, res);
    return res;
  }
};
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef SOT_CORE_BINARYOP_HH
