/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_TASK_HH__
#define __SOT_FEATURE_TASK_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/exception-task.hh>
#include <sot/core/feature-generic.hh>
#include <sot/core/task-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(feature_task_EXPORTS)
#define SOTFEATURETASK_EXPORT __declspec(dllexport)
#else
#define SOTFEATURETASK_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFEATURETASK_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

class SOTFEATURETASK_EXPORT FeatureTask : public FeatureGeneric {
 public:
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 protected:
  TaskAbstract *taskPtr;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
 public:
  /*! \brief Default constructor */
  FeatureTask(const std::string &name);

  /*! \brief Default destructor */
  virtual ~FeatureTask(void) {}

  /*! \brief Display the information related to this task implementation. */
  virtual void display(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_FEATURE_TASK_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
