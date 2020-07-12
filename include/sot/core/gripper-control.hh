/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTGRIPPERCONTROL_H__
#define __SOT_SOTGRIPPERCONTROL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/flags.hh>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(gripper_control_EXPORTS)
#define SOTGRIPPERCONTROL_EXPORT __declspec(dllexport)
#else
#define SOTGRIPPERCONTROL_EXPORT __declspec(dllimport)
#endif
#else
#define SOTGRIPPERCONTROL_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

namespace dg = dynamicgraph;

/*! \brief The goal of this entity is to ensure that the maximal torque will not
 * be exceeded during a grasping task.
 * If the maximal torque is reached, then the current position of the gripper is
kept
 *
 */
class SOTGRIPPERCONTROL_EXPORT GripperControl {
protected:
  double offset;
  static const double OFFSET_DEFAULT;
  //! \brief The multiplication
  dg::Vector factor;

public:
  GripperControl(void);

  //! \brief Computes the
  // if the torque limit is reached, the normalized position is reduced by
  // (offset)
  void computeIncrement(const dg::Vector &torques,
                        const dg::Vector &torqueLimits,
                        const dg::Vector &currentNormVel);

  //! \brief
  dg::Vector &computeDesiredPosition(const dg::Vector &currentPos,
                                     const dg::Vector &desiredPos,
                                     const dg::Vector &torques,
                                     const dg::Vector &torqueLimits,
                                     dg::Vector &referencePos);

  /*! \brief select only some of the values of the vector fullsize,
   *   based on the Flags vector.
   */

  static dg::Vector &selector(const dg::Vector &fullsize, const Flags &selec,
                              dg::Vector &desPos);
};

/* --------------------------------------------------------------------- */
/* --- PLUGIN ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTGRIPPERCONTROL_EXPORT GripperControlPlugin : public dg::Entity,
                                                      public GripperControl {
  DYNAMIC_GRAPH_ENTITY_DECL();

public:
  bool calibrationStarted;

public: /* --- CONSTRUCTION --- */
  GripperControlPlugin(const std::string &name);
  virtual ~GripperControlPlugin(void);

  /* --- DOCUMENTATION --- */
  virtual std::string getDocString() const;

public: /* --- SIGNAL --- */
  /* --- INPUTS --- */
  dg::SignalPtr<dg::Vector, int> positionSIN;
  dg::SignalPtr<dg::Vector, int> positionDesSIN;
  dg::SignalPtr<dg::Vector, int> torqueSIN;
  dg::SignalPtr<dg::Vector, int> torqueLimitSIN;
  dg::SignalPtr<Flags, int> selectionSIN;

  /* --- INTERMEDIARY --- */
  dg::SignalPtr<dg::Vector, int> positionFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector, int> positionReduceSOUT;
  dg::SignalPtr<dg::Vector, int> torqueFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector, int> torqueReduceSOUT;
  dg::SignalPtr<dg::Vector, int> torqueLimitFullSizeSIN;
  dg::SignalTimeDependent<dg::Vector, int> torqueLimitReduceSOUT;

  /* --- OUTPUTS --- */
  dg::SignalTimeDependent<dg::Vector, int> desiredPositionSOUT;

public: /* --- COMMANDLINE --- */
  void initCommands();

  void setOffset(const double &value);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_SOTGRIPPERCONTROL_H__
