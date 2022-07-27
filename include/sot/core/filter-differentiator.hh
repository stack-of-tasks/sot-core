/*
 * Copyright 2017-, Rohan Budhirja, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_torque_control_FilterDifferentiator_H__
#define __sot_torque_control_FilterDifferentiator_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(low_pass_filter_EXPORTS)
#define SOTFILTERDIFFERENTIATOR_EXPORT __declspec(dllexport)
#else
#define SOTFILTERDIFFERENTIATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFILTERDIFFERENTIATOR_EXPORT
#endif

//#define VP_DEBUG 1        /// enable debug output
//#define VP_DEBUG_MODE 20

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* HELPER */
#include <dynamic-graph/signal-helper.h>

#include <sot/core/causal-filter.hh>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
/** \addtogroup Filters
    \section subsec_filterdiff FilterDifferentiator
  This Entity takes as inputs a signal and applies a low pass filter
  (implemented through CasualFilter) and computes finite difference derivative.

  The input signal is provided through m_xSIN (an entity signal).
  The filtered signal is given through m_x_filteredSOUT.
  The first derivative of the filtered signal is provided with m_dxSOUT.
  The second derivative of the filtered signal is provided with m_ddxSOUT.
  */
class SOTFILTERDIFFERENTIATOR_EXPORT FilterDifferentiator
    : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public: /* --- SIGNALS --- */
         /// Input signals
  DECLARE_SIGNAL_IN(x, dynamicgraph::Vector);
  /// Output signal x_filtered
  DECLARE_SIGNAL_OUT(x_filtered, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dx, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(ddx, dynamicgraph::Vector);

  /// The following inner signals are used because this entity has
  /// some output signals
  /// whose related quantities are computed at the same time by the
  /// same algorithm
  /// To avoid the risk of recomputing the same things twice,
  /// we create an inner signal that groups together
  /// all the quantities that are computed together.
  /// Then the single output signals will depend
  /// on this inner signal, which is the one triggering the computations.
  /// Inner signals are not exposed, so that nobody can access them.

  /// This signal contains the estimated positions, velocities and
  /// accelerations.
  DECLARE_SIGNAL_INNER(x_dx_ddx, dynamicgraph::Vector);

 protected:
  double m_dt;  /// sampling timestep of the input signal
  int m_x_size;

  /// polynomial-fitting filters
  CausalFilter *m_filter;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** --- CONSTRUCTOR ---- */
  FilterDifferentiator(const std::string &name);

  /** Initialize the FilterDifferentiator.
   * @param timestep Period (in seconds) after which
   * the sensors' data are updated.
   * @param sigSize  Size of the input signal.
   * @param delay    Delay (in seconds) introduced by the estimation.
   *                 This should be a multiple of timestep.
   * @note The estimationDelay is half of the length of the
   * window used for the
   * polynomial fitting. The larger the delay,
   * the smoother the estimations.
   */
  void init(const double &timestep, const int &xSize,
            const Eigen::VectorXd &filter_numerator,
            const Eigen::VectorXd &filter_denominator);

  void switch_filter(const Eigen::VectorXd &filter_numerator,
                     const Eigen::VectorXd &filter_denominator);

 protected:
 public: /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream &os) const;

};  // class FilterDifferentiator

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_FilterDifferentiator_H__
