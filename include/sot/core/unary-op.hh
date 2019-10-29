/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_CORE_UNARYOP_HH
#define SOT_CORE_UNARYOP_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

template <typename Operator> class UnaryOp : public Entity {
  Operator op;
  typedef typename Operator::Tin Tin;
  typedef typename Operator::Tout Tout;
  typedef UnaryOp<Operator> Self;

public: /* --- CONSTRUCTION --- */
  static std::string getTypeInName(void) { return Operator::nameTypeIn(); }
  static std::string getTypeOutName(void) { return Operator::nameTypeOut(); }
  static const std::string CLASS_NAME;

  virtual const std::string &getClassName() const { return CLASS_NAME; }

  std::string getDocString() const { return op.getDocString(); }

  UnaryOp(const std::string &name)
      : Entity(name), SIN(NULL, Self::CLASS_NAME + "(" + name + ")::input(" +
                                    Self::getTypeInName() + ")::sin"),
        SOUT(boost::bind(&Self::computeOperation, this, _1, _2), SIN,
             Self::CLASS_NAME + "(" + name + ")::output(" +
                 Self::getTypeOutName() + ")::sout") {
    signalRegistration(SIN << SOUT);
    op.addSpecificCommands(*this, commandMap);
  }

  virtual ~UnaryOp(void){};

public: /* --- SIGNAL --- */
  SignalPtr<Tin, int> SIN;
  SignalTimeDependent<Tout, int> SOUT;

protected:
  Tout &computeOperation(Tout &res, int time) {
    const Tin &x1 = SIN(time);
    op(x1, res);
    return res;
  }

public: /* --- PARAMS --- */
};
} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef SOT_CORE_UNARYOP_HH
