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
  dynamicgraph::Vector factor;

 public:
  GripperControl(void);

  //! \brief Computes the
  // if the torque limit is reached, the normalized position is reduced by
  // (offset)
  void computeIncrement(const dynamicgraph::Vector &torques,
                        const dynamicgraph::Vector &torqueLimits,
                        const dynamicgraph::Vector &currentNormVel);

  //! \brief
  dynamicgraph::Vector &computeDesiredPosition(
      const dynamicgraph::Vector &currentPos,
      const dynamicgraph::Vector &desiredPos,
      const dynamicgraph::Vector &torques,
      const dynamicgraph::Vector &torqueLimits,
      dynamicgraph::Vector &referencePos);

  /*! \brief select only some of the values of the vector fullsize,
   *   based on the Flags vector.
   */

  static dynamicgraph::Vector &selector(const dynamicgraph::Vector &fullsize,
                                        const Flags &selec,
                                        dynamicgraph::Vector &desPos);
};

/* --------------------------------------------------------------------- */
/* --- PLUGIN ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTGRIPPERCONTROL_EXPORT GripperControlPlugin
    : public dynamicgraph::Entity,
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
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> positionSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> positionDesSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> torqueSIN;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> torqueLimitSIN;
  dynamicgraph::SignalPtr<Flags, int> selectionSIN;

  /* --- INTERMEDIARY --- */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> positionFullSizeSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
      positionReduceSOUT;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> torqueFullSizeSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> torqueReduceSOUT;
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> torqueLimitFullSizeSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
      torqueLimitReduceSOUT;

  /* --- OUTPUTS --- */
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
      desiredPositionSOUT;

 public: /* --- COMMANDLINE --- */
  void initCommands();

  void setOffset(const double &value);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_SOTGRIPPERCONTROL_H__
