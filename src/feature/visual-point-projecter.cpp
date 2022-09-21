/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 */

#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include <sot/core/visual-point-projecter.hh>

namespace dynamicgraph {
namespace sot {

namespace dg = ::dynamicgraph;
using namespace dg;

/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VisualPointProjecter,
                                   "VisualPointProjecter");

/* --- CONSTRUCTION ----------------------------------------------------- */
/* --- CONSTRUCTION ----------------------------------------------------- */
/* --- CONSTRUCTION ----------------------------------------------------- */
VisualPointProjecter::VisualPointProjecter(const std::string &name)
    : Entity(name)

      ,
      CONSTRUCT_SIGNAL_IN(point3D, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(transfo, MatrixHomogeneous)

      ,
      CONSTRUCT_SIGNAL_OUT(point3Dgaze, dynamicgraph::Vector,
                           m_point3DSIN << m_transfoSIN),
      CONSTRUCT_SIGNAL_OUT(depth, double, m_point3DgazeSOUT),
      CONSTRUCT_SIGNAL_OUT(point2D, dynamicgraph::Vector,
                           m_point3DgazeSOUT << m_depthSOUT) {
  Entity::signalRegistration(m_point3DSIN);
  Entity::signalRegistration(m_transfoSIN);
  Entity::signalRegistration(m_point3DgazeSOUT);
  Entity::signalRegistration(m_point2DSOUT);
  Entity::signalRegistration(m_depthSOUT);
}

/* --- SIGNALS ---------------------------------------------------------- */
/* --- SIGNALS ---------------------------------------------------------- */
/* --- SIGNALS ---------------------------------------------------------- */

dynamicgraph::Vector &VisualPointProjecter::point3DgazeSOUT_function(
    dynamicgraph::Vector &p3g, int iter) {
  const dynamicgraph::Vector &p3 = m_point3DSIN(iter);
  const MatrixHomogeneous &M = m_transfoSIN(iter);
  MatrixHomogeneous Mi;
  Mi = M.inverse(Eigen::Affine);
  p3g = Mi.matrix() * p3;
  return p3g;
}

dynamicgraph::Vector &VisualPointProjecter::point2DSOUT_function(
    dynamicgraph::Vector &p2, int iter) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector &p3 = m_point3DgazeSOUT(iter);
  const double &z = m_depthSOUT(iter);
  assert(z > 0);

  p2.resize(2);
  p2(0) = p3(0) / z;
  p2(1) = p3(1) / z;

  sotDEBUGOUT(15);
  return p2;
}

double &VisualPointProjecter::depthSOUT_function(double &z, int iter) {
  const dynamicgraph::Vector &p3 = m_point3DgazeSOUT(iter);
  assert(p3.size() == 3);
  z = p3(2);
  return z;
}

/* --- ENTITY ----------------------------------------------------------- */
/* --- ENTITY ----------------------------------------------------------- */
/* --- ENTITY ----------------------------------------------------------- */

void VisualPointProjecter::display(std::ostream &os) const {
  os << "VisualPointProjecter " << getName();
}

}  // namespace sot
}  // namespace dynamicgraph
