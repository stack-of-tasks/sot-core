/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_JOINTLIMITS_HH__
#define __SOT_JOINTLIMITS_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/exception-task.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(motion_period_EXPORTS)
#define SOTMOTIONPERIOD_EXPORT __declspec(dllexport)
#else
#define SOTMOTIONPERIOD_EXPORT __declspec(dllimport)
#endif
#else
#define SOTMOTIONPERIOD_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*!
  \class MotionPeriod
*/
namespace dynamicgraph {
namespace sot {

namespace dg = dynamicgraph;

class SOTMOTIONPERIOD_EXPORT MotionPeriod : public dg::Entity {

public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  enum MotionPeriodType { MOTION_CONSTANT, MOTION_SIN, MOTION_COS };

  struct sotMotionParam {
    MotionPeriodType motionType;
    unsigned int period;
    unsigned int initPeriod;
    double amplitude;
    double initAmplitude;
  };

  unsigned int size;
  std::vector<sotMotionParam> motionParams;

  void resize(const unsigned int &size);

  /* --- SIGNALS ------------------------------------------------------------ */
public:
  dg::SignalTimeDependent<dg::Vector, int> motionSOUT;

public:
  MotionPeriod(const std::string &name);
  virtual ~MotionPeriod(void) {}

  dg::Vector &computeMotion(dg::Vector &res, const int &time);

  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_JOINTLIMITS_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
