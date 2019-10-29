#ifndef _SOT_CORE_CAUSAL_FILTER_H_
#define _SOT_CORE_CAUSAL_FILTER_H_
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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <Eigen/Core>

/** \addtogroup Filters
    \section subsec_causalfilter CausalFilter
    Filter data with an IIR or FIR filter.

    Filter a data sequence, \f$x\f$, using a digital filter.
    The filter is a direct form II transposed implementation
    of the standard difference equation.
    This means that the filter implements:

    \f$ a[0]*y[N] = b[0]*x[N] + b[1]*x[N-1] + ... + b[m-1]*x[N-(m-1)]
    - a[1]*y[N-1] - ... - a[n-1]*y[N-(n-1)] \f$

    where \f$m\f$ is the degree of the numerator,
    \f$n\f$ is the degree of the denominator,
    and \f$N\f$ is the sample number


 */
namespace dynamicgraph {
namespace sot {

class CausalFilter {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** --- CONSTRUCTOR ----
      \param[in] timestep
      \param[in] xSize
      \param[in] filter_numerator
      \param[in] filter_denominator

      xSize is
  */
  CausalFilter(const double &timestep, const int &xSize,
               const Eigen::VectorXd &filter_numerator,
               const Eigen::VectorXd &filter_denominator);

  void get_x_dx_ddx(const Eigen::VectorXd &base_x,
                    Eigen::VectorXd &x_output_dx_ddx);

  void switch_filter(const Eigen::VectorXd &filter_numerator,
                     const Eigen::VectorXd &filter_denominator);

private:
  /// sampling timestep of the input signal
  double m_dt;
  /// Size
  int m_x_size;
  /// Size of the numerator \f$m\f$
  Eigen::VectorXd::Index m_filter_order_m;
  /// Size of the denominator \f$n\f$
  Eigen::VectorXd::Index m_filter_order_n;

  /// Coefficients of the numerator \f$b\f$
  Eigen::VectorXd m_filter_numerator;
  /// Coefficients of the denominator \f$a\f$
  Eigen::VectorXd m_filter_denominator;
  bool m_first_sample;
  ///
  int m_pt_numerator;
  int m_pt_denominator;
  Eigen::MatrixXd m_input_buffer;
  Eigen::MatrixXd m_output_buffer;
}; // class CausalFilter
} // namespace sot
} // namespace dynamicgraph
#endif /* _SOT_CORE_CAUSAL_FILTER_H_ */
