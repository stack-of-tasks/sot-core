/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOTSEQUENCER_H__
#define __SOT_SOTSEQUENCER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <sot/core/task-abstract.hh>

/* STD */
#include <list>
#include <map>
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(sequencer_EXPORTS)
#define SOTSEQUENCER_EXPORT __declspec(dllexport)
#else
#define SOTSEQUENCER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTSEQUENCER_EXPORT
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class Sot;

class SOTSEQUENCER_EXPORT Sequencer : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  class sotEventAbstract {
   public:
    enum sotEventType { EVENT_ADD, EVENT_RM, EVENT_CMD };

   protected:
    std::string name;
    void setName(const std::string &name_) { name = name_; }
    size_type eventType;

   public:
    sotEventAbstract(const std::string &name) : name(name) {};
    virtual ~sotEventAbstract(void) {}
    virtual const std::string &getName() const { return name; }
    size_type getEventType() const { return eventType; }
    virtual void operator()(Sot *sotPtr) = 0;
    virtual void display(std::ostream &os) const { os << name; }
  };

 protected:
  Sot *sotPtr;
  typedef std::list<sotEventAbstract *> TaskList;
  typedef std::map<std::size_t, TaskList> TaskMap;

  TaskMap taskMap;
  /* All the events are counting wrt to this t0. If t0 is -1, it
   * is set to the first time of trig.    */
  sigtime_t timeInit;
  bool playMode;
  std::ostream *outputStreamPtr;
  bool noOutput; /*! if true, display nothing standard output on except errors*/

 public: /* --- CONSTRUCTION --- */
  Sequencer(const std::string &name);
  virtual ~Sequencer(void);

 public: /* --- TASK MANIP --- */
  void setSotRef(Sot *sot) { sotPtr = sot; }
  void addTask(sotEventAbstract *task, const std::size_t time);
  void rmTask(size_type eventType, const std::string &name,
              const std::size_t time);
  void clearAll();

 public: /* --- SIGNAL --- */
  dynamicgraph::SignalTimeDependent<size_type, sigtime_t> triggerSOUT;

 public: /* --- FUNCTIONS --- */
  size_type &trigger(size_type &dummy, const sigtime_t &time);

 public: /* --- PARAMS --- */
  virtual void display(std::ostream &os) const;
};
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __SOT_SOTSEQUENCER_H__
