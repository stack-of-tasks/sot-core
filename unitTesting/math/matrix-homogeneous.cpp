// Copyright 2011 Florent Lamiraux
//
// This file is part of sot-core.
// sot-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// sot-core is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with sot-core.  If not, see <http://www.gnu.org/licenses/>.

#include <sstream>

#define BOOST_TEST_MODULE matrix_homogeneous

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/output_test_stream.hpp>

#include <jrl/mal/boost.hh>
#include <sot/core/matrix-homogeneous.hh>
#include "sot/core/matrix-twist.hh"

using boost::test_tools::output_test_stream;

namespace ml = maal::boost;

#define MATRIX_BOOST_REQUIRE_CLOSE(N, M, LEFT, RIGHT, TOLERANCE)	\
  for (unsigned i = 0; i < N; ++i)					\
    for (unsigned j = 0; j < M; ++j)					\
      BOOST_REQUIRE_CLOSE(LEFT (i, j), RIGHT (i, j), TOLERANCE)

#define MATRIX_4x4_BOOST_REQUIRE_CLOSE(LEFT, RIGHT, TOLERANCE)	\
  MATRIX_BOOST_REQUIRE_CLOSE (4, 4, LEFT, RIGHT, TOLERANCE)

#define MATRIX_HOMO_INIT(M,						\
			 tx, ty, tz,					\
			 roll, pitch, yaw)				\
  M (0, 0) = cos(pitch)*cos(yaw);					\
  M (0, 1) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);	\
  M (0, 2) = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);	\
  M (0, 3) = tx;							\
  M (1, 0) = cos(pitch)*sin(yaw);					\
  M (1, 1) = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);	\
  M (1, 2) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);	\
  M (1, 3) = ty;							\
  M (2, 0) = -sin(pitch);						\
  M (2, 1) = sin(roll)*cos(pitch);					\
  M (2, 2) = cos(roll)*cos(pitch);					\
  M (2, 3) = tz;							\
  M (3, 0) = 0.; M (3, 1) = 0.; M (3, 2) = 0.; M (3, 3) = 1.		\

BOOST_AUTO_TEST_CASE (product)
{
  for (unsigned int i=0; i<1000; i++) {
    double tx, ty, tz;
    double roll, pitch, yaw;
    dynamicgraph::sot::MatrixHomogeneous H1;
    tx = (10.*rand())/RAND_MAX - 5.;
    ty = (10.*rand())/RAND_MAX - 5.;
    tz = (10.*rand())/RAND_MAX - 5.;
    roll = (M_PI*rand())/RAND_MAX - .5*M_PI;
    pitch = (M_PI*rand())/RAND_MAX - .5*M_PI;
    yaw = (2*M_PI*rand())/RAND_MAX - M_PI;
    MATRIX_HOMO_INIT(H1, tx, ty, tz, roll, pitch, yaw);
    ml::Matrix M1(H1);
    dynamicgraph::sot::MatrixHomogeneous H2;
    tx = (10.*rand())/RAND_MAX;
    ty = (10.*rand())/RAND_MAX - 5.;
    tz = (10.*rand())/RAND_MAX - 5.;
    roll = (M_PI*rand())/RAND_MAX - .5*M_PI;
    pitch = (M_PI*rand())/RAND_MAX - .5*M_PI;
    yaw = (2*M_PI*rand())/RAND_MAX - M_PI;
    MATRIX_HOMO_INIT(H2, tx, ty, tz, roll, pitch, yaw);
    ml::Matrix M2(H2);
    dynamicgraph::sot::MatrixHomogeneous H3 = H1*H2;
    ml::Matrix M3 = M1*M2;

    MATRIX_4x4_BOOST_REQUIRE_CLOSE (M3, H3, 0.001);
  }
}
