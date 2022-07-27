/*
 * Copyright 2017-, Rohan Budhiraja LAAS-CNRS
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

#define LOGFILE "/tmp/fd_log.dat"

#define LOG(x)                                 \
  {                                            \
    std::ofstream LogFile;                     \
    LogFile.open(LOGFILE, std::ofstream::app); \
    LogFile << x << std::endl;                 \
    LogFile.close();                           \
  }

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include <sot/core/filter-differentiator.hh>
//#include <sot/torque_control/motor-model.hh>
#include <Eigen/Dense>

namespace dynamicgraph {
namespace sot {

#define ALL_INPUT_SIGNALS m_xSIN

#define ALL_OUTPUT_SIGNALS m_x_filteredSOUT << m_dxSOUT << m_ddxSOUT

namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace Eigen;

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef FilterDifferentiator EntityClassName;

/* --- DG FACTORY --------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FilterDifferentiator,
                                   "FilterDifferentiator");

/* --- CONSTRUCTION ------------------------------------------------- */
/* --- CONSTRUCTION ------------------------------------------------- */
/* --- CONSTRUCTION ------------------------------------------------- */
FilterDifferentiator::FilterDifferentiator(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(x, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(x_filtered, dynamicgraph::Vector, m_x_dx_ddxSINNER),
      CONSTRUCT_SIGNAL_OUT(dx, dynamicgraph::Vector, m_x_dx_ddxSINNER),
      CONSTRUCT_SIGNAL_OUT(ddx, dynamicgraph::Vector, m_x_dx_ddxSINNER),
      CONSTRUCT_SIGNAL_INNER(x_dx_ddx, dynamicgraph::Vector, m_xSIN) {
  Entity::signalRegistration(ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);

  /* Commands. */
  addCommand(
      "getTimestep",
      makeDirectGetter(*this, &m_dt,
                       docDirectGetter("Control timestep [s ]", "double")));
  addCommand("getSize",
             makeDirectGetter(*this, &m_x_size,
                              docDirectGetter("Size of the x signal", "int")));
  addCommand("init",
             makeCommandVoid4(*this, &FilterDifferentiator::init,
                              docCommandVoid4("Initialize the filter.",
                                              "Control timestep [s].",
                                              "Size of the input signal x",
                                              "Numerator of the filter",
                                              "Denominator of the filter")));
  addCommand("switch_filter",
             makeCommandVoid2(
                 *this, &FilterDifferentiator::switch_filter,
                 docCommandVoid2("Switch Filter.", "Numerator of the filter",
                                 "Denominator of the filter")));
}

/* --- COMMANDS ------------------------------------------------------ */
/* --- COMMANDS ------------------------------------------------------ */
/* --- COMMANDS ------------------------------------------------------ */
void FilterDifferentiator::init(const double &timestep, const int &xSize,
                                const Eigen::VectorXd &filter_numerator,
                                const Eigen::VectorXd &filter_denominator) {
  m_x_size = xSize;
  m_dt = timestep;
  m_filter =
      new CausalFilter(timestep, xSize, filter_numerator, filter_denominator);

  LOG("Filtering started with "
      << "Numerator " << filter_numerator << std::endl
      << "Denominator" << filter_denominator << std::endl);
  return;
}

void FilterDifferentiator::switch_filter(
    const Eigen::VectorXd &filter_numerator,
    const Eigen::VectorXd &filter_denominator) {
  LOG("Filter switched with "
      << "Numerator " << filter_numerator << std::endl
      << "Denominator" << filter_denominator << std::endl
      << "at time" << m_xSIN.getTime());
  m_filter->switch_filter(filter_numerator, filter_denominator);
}

/* --- SIGNALS ------------------------------------------------------ */
/* --- SIGNALS ------------------------------------------------------ */
/* --- SIGNALS ------------------------------------------------------ */

DEFINE_SIGNAL_INNER_FUNCTION(x_dx_ddx, dynamicgraph::Vector) {
  sotDEBUG(15) << "Compute x_dx inner signal " << iter << std::endl;
  if (s.size() != 3 * m_x_size) s.resize(3 * m_x_size);
  // read encoders
  const dynamicgraph::Vector &base_x = m_xSIN(iter);
  assert(base_x.size() == m_x_size);
  m_filter->get_x_dx_ddx(base_x, s);
  return s;
}

/// *************************************************************** ///
/// The following signals depend only on other inner signals, so they
/// just need to copy the interested part of the inner signal
/// they depend on.
/// *************************************************************** ///

DEFINE_SIGNAL_OUT_FUNCTION(x_filtered, dynamicgraph::Vector) {
  sotDEBUG(15) << "Compute x_filtered output signal " << iter << std::endl;

  const dynamicgraph::Vector &x_dx_ddx = m_x_dx_ddxSINNER(iter);
  if (s.size() != m_x_size) s.resize(m_x_size);
  s = x_dx_ddx.head(m_x_size);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dx, dynamicgraph::Vector) {
  sotDEBUG(15) << "Compute dx output signal " << iter << std::endl;

  const dynamicgraph::Vector &x_dx_ddx = m_x_dx_ddxSINNER(iter);
  if (s.size() != m_x_size) s.resize(m_x_size);
  s = x_dx_ddx.segment(m_x_size, m_x_size);
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(ddx, dynamicgraph::Vector) {
  sotDEBUG(15) << "Compute ddx output signal " << iter << std::endl;

  const dynamicgraph::Vector &x_dx_ddx = m_x_dx_ddxSINNER(iter);
  if (s.size() != m_x_size) s.resize(m_x_size);
  s = x_dx_ddx.tail(m_x_size);
  return s;
}

void FilterDifferentiator::display(std::ostream &os) const {
  os << "FilterDifferentiator " << getName() << ":\n";
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace sot
}  // namespace dynamicgraph
