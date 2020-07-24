/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 */

#ifndef __sot_core_VisualPointProjecter_H__
#define __sot_core_VisualPointProjecter_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(visual_point_projecter_EXPORTS)
#define SOTVISUALPOINTPROJECTER_EXPORT __declspec(dllexport)
#else
#define SOTVISUALPOINTPROJECTER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTVISUALPOINTPROJECTER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

/* SOT */
#include <dynamic-graph/entity-helper.h>
#include <dynamic-graph/signal-helper.h>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTVISUALPOINTPROJECTER_EXPORT VisualPointProjecter
    : public ::dynamicgraph::Entity,
      public ::dynamicgraph::EntityHelper<VisualPointProjecter> {

public: /* --- CONSTRUCTOR ---- */
  VisualPointProjecter(const std::string &name);

public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

public: /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(point3D, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(transfo, MatrixHomogeneous);

  DECLARE_SIGNAL_OUT(point3Dgaze, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(depth, double);
  DECLARE_SIGNAL_OUT(point2D, dynamicgraph::Vector);

}; // class VisualPointProjecter

} // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __sot_core_VisualPointProjecter_H__
