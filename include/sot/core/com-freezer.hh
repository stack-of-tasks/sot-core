/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTCOMFREEZER_H_H
#define __SOT_SOTCOMFREEZER_H_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(com_freezer_EXPORTS)
#define SOTCOMFREEZER_EXPORT __declspec(dllexport)
#else
#define SOTCOMFREEZER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTCOMFREEZER_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTCOMFREEZER_EXPORT CoMFreezer : public dynamicgraph::Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName() const { return CLASS_NAME; }

 private:
  dynamicgraph::Vector m_lastCoM;
  bool m_previousPGInProcess;
  int m_lastStopTime;

 public: /* --- CONSTRUCTION --- */
  CoMFreezer(const std::string &name);
  virtual ~CoMFreezer(void);

 public: /* --- SIGNAL --- */
  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> CoMRefSIN;
  dynamicgraph::SignalPtr<unsigned, int> PGInProcessSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> freezedCoMSOUT;

 public: /* --- FUNCTION --- */
  dynamicgraph::Vector &computeFreezedCoM(dynamicgraph::Vector &freezedCoM,
                                          const int &time);

 public: /* --- PARAMS --- */
  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_SOTCOMFREEZER_H_H */
