/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_PERIODICCALL_ENTITY_HH__
#define __SOT_PERIODICCALL_ENTITY_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/periodic-call-entity.hh>
#include <sot/core/periodic-call.hh>
/* STD */
#include <list>
#include <map>
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(periodic_call_entity_EXPORTS)
#define PeriodicCallEntity_EXPORT __declspec(dllexport)
#else
#define PeriodicCallEntity_EXPORT __declspec(dllimport)
#endif
#else
#define PeriodicCallEntity_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*!
  \class PeriodicCallEntity

  The entity remembers a stack of signal and command to be executed or
  refreshed at each iteration. The update is trigered by the triger signal.
  If the trigerOnce is called, the stacks are flushed after the execution.
*/
class PeriodicCallEntity_EXPORT PeriodicCallEntity
    : public Entity,
      protected sot::PeriodicCall {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  Signal<int, int> triger;
  Signal<int, int> trigerOnce;

  int &trigerCall(int &dummy, const int &time);
  int &trigerOnceCall(int &dummy, const int &time);

  /* --- FUNCTIONS ------------------------------------------------------------
   */
 public:
  PeriodicCallEntity(const std::string &name);
  virtual ~PeriodicCallEntity(void) {}

  virtual void display(std::ostream &os) const;
};

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_PERIODICCALL_ENTITY_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
