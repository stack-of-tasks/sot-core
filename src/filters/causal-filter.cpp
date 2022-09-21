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

#include <iostream>
#include <sot/core/causal-filter.hh>

using namespace dynamicgraph::sot;
/*
Filter data with an IIR or FIR filter.

Filter a data sequence, x, using a digital filter. The filter is a direct form
II transposed implementation of the standard difference equation. This means
that the filter implements:

a[0]*y[N] = b[0]*x[N] + b[1]*x[N-1] + ... + b[m-1]*x[N-(m-1)]
                      - a[1]*y[N-1] - ... - a[n-1]*y[N-(n-1)]

where m is the degree of the numerator, n is the degree of the denominator, and
N is the sample number

*/

CausalFilter::CausalFilter(const double &timestep, const int &xSize,
                           const Eigen::VectorXd &filter_numerator,
                           const Eigen::VectorXd &filter_denominator)

    : m_dt(timestep),
      m_x_size(xSize),
      m_filter_order_m(filter_numerator.size()),
      m_filter_order_n(filter_denominator.size()),
      m_filter_numerator(filter_numerator),
      m_filter_denominator(filter_denominator),
      m_first_sample(true),
      m_pt_numerator(0),
      m_pt_denominator(0),
      m_input_buffer(Eigen::MatrixXd::Zero(xSize, filter_numerator.size())),
      m_output_buffer(
          Eigen::MatrixXd::Zero(xSize, filter_denominator.size() - 1)) {
  assert(timestep > 0.0 && "Timestep should be > 0");
  assert(m_filter_numerator.size() == m_filter_order_m);
  assert(m_filter_denominator.size() == m_filter_order_n);
}

void CausalFilter::get_x_dx_ddx(const Eigen::VectorXd &base_x,
                                Eigen::VectorXd &x_output_dx_ddx) {
  // const dynamicgraph::Vector &base_x = m_xSIN(iter);
  if (m_first_sample) {
    for (int i = 0; i < m_filter_order_m; i++) m_input_buffer.col(i) = base_x;
    for (int i = 0; i < m_filter_order_n - 1; i++)
      m_output_buffer.col(i) =
          base_x * m_filter_numerator.sum() / m_filter_denominator.sum();
    m_first_sample = false;
  }

  m_input_buffer.col(m_pt_numerator) = base_x;

  Eigen::VectorXd b(m_filter_order_m);
  Eigen::VectorXd a(m_filter_order_n - 1);
  b.head(m_pt_numerator + 1) =
      m_filter_numerator.head(m_pt_numerator + 1).reverse();
  b.tail(m_filter_order_m - m_pt_numerator - 1) =
      m_filter_numerator.tail(m_filter_order_m - m_pt_numerator - 1).reverse();

  a.head(m_pt_denominator + 1) =
      m_filter_denominator.segment(1, m_pt_denominator + 1).reverse();
  a.tail(m_filter_order_n - m_pt_denominator - 2) =
      m_filter_denominator.tail(m_filter_order_n - m_pt_denominator - 2)
          .reverse();
  x_output_dx_ddx.head(m_x_size) =
      (m_input_buffer * b - m_output_buffer * a) / m_filter_denominator[0];

  // Finite Difference
  Eigen::VectorXd::Index m_pt_denominator_prev =
      (m_pt_denominator == 0) ? m_filter_order_n - 2 : m_pt_denominator - 1;
  x_output_dx_ddx.segment(m_x_size, m_x_size) =
      (x_output_dx_ddx.head(m_x_size) - m_output_buffer.col(m_pt_denominator)) /
      m_dt;
  x_output_dx_ddx.tail(m_x_size) =
      (x_output_dx_ddx.head(m_x_size) -
       2 * m_output_buffer.col(m_pt_denominator) +
       m_output_buffer.col(m_pt_denominator_prev)) /
      m_dt / m_dt;

  m_pt_numerator =
      (m_pt_numerator + 1) < m_filter_order_m ? (m_pt_numerator + 1) : 0;
  m_pt_denominator = (m_pt_denominator + 1) < m_filter_order_n - 1
                         ? (m_pt_denominator + 1)
                         : 0;
  m_output_buffer.col(m_pt_denominator) = x_output_dx_ddx.head(m_x_size);
  return;
}

void CausalFilter::switch_filter(const Eigen::VectorXd &filter_numerator,
                                 const Eigen::VectorXd &filter_denominator) {
  Eigen::VectorXd::Index filter_order_m = filter_numerator.size();
  Eigen::VectorXd::Index filter_order_n = filter_denominator.size();

  Eigen::VectorXd current_x(m_input_buffer.col(m_pt_numerator));

  m_input_buffer.resize(Eigen::NoChange, filter_order_m);
  m_output_buffer.resize(Eigen::NoChange, filter_order_n - 1);

  for (int i = 0; i < filter_order_m; i++) m_input_buffer.col(i) = current_x;

  for (int i = 0; i < filter_order_n - 1; i++)
    m_output_buffer.col(i) =
        current_x * filter_numerator.sum() / filter_denominator.sum();

  m_filter_order_m = filter_order_m;
  m_filter_numerator.resize(filter_order_m);
  m_filter_numerator = filter_numerator;

  m_filter_order_n = filter_order_n;
  m_filter_denominator.resize(filter_order_n);
  m_filter_denominator = filter_denominator;

  m_pt_numerator = 0;
  m_pt_denominator = 0;

  return;
}
