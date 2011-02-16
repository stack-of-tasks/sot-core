// Copyright 2011 Thomas Moulard.
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

#define BOOST_TEST_MODULE matrix_twist

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

#define MATRIX_6x6_BOOST_REQUIRE_CLOSE(LEFT, RIGHT, TOLERANCE)	\
  MATRIX_BOOST_REQUIRE_CLOSE (6, 6, LEFT, RIGHT, TOLERANCE)

#define MATRIX_4x4_INIT(M,					  \
			A00, A01, A02, A03,			  \
			A10, A11, A12, A13,			  \
			A20, A21, A22, A23,			  \
			A30, A31, A32, A33)			  \
  M (0, 0) = A00, M (0, 1) = A01, M (0, 2) = A02, M (0, 3) = A03; \
  M (1, 0) = A10, M (1, 1) = A11, M (1, 2) = A12, M (1, 3) = A13; \
  M (2, 0) = A20, M (2, 1) = A21, M (2, 2) = A22, M (2, 3) = A23; \
  M (3, 0) = A30, M (3, 1) = A31, M (3, 2) = A32, M (3, 3) = A33  \

#define MATRIX_6x6_INIT(M,						\
			A00, A01, A02, A03, A04, A05,			\
			A10, A11, A12, A13, A14, A15,			\
			A20, A21, A22, A23, A24, A25,			\
			A30, A31, A32, A33, A34, A35,			\
			A40, A41, A42, A43, A44, A45,			\
			A50, A51, A52, A53, A54, A55)			\
  M (0, 0) = A00, M (0, 1) = A01, M (0, 2) = A02, M (0, 3) = A03,	\
    M (0, 4) = A04, M (0, 5) = A05;					\
  M (1, 0) = A10, M (1, 1) = A11, M (1, 2) = A12, M (1, 3) = A13,	\
    M (1, 4) = A14, M (1, 5) = A15;					\
  M (2, 0) = A20, M (2, 1) = A21, M (2, 2) = A22, M (2, 3) = A23,	\
    M (2, 4) = A24, M (2, 5) = A25;					\
  M (3, 0) = A30, M (3, 1) = A31, M (3, 2) = A32, M (3, 3) = A33,	\
    M (3, 4) = A34, M (3, 5) = A35;					\
  M (4, 0) = A40, M (4, 1) = A41, M (4, 2) = A42, M (4, 3) = A43,	\
    M (4, 4) = A44, M (4, 5) = A45;					\
  M (5, 0) = A50, M (5, 1) = A51, M (5, 2) = A52, M (5, 3) = A53,	\
    M (5, 4) = A54, M (5, 5) = A55



BOOST_AUTO_TEST_CASE (constructor_trivial)
{
  dynamicgraph::sot::MatrixHomogeneous M;
  dynamicgraph::sot::MatrixTwist twist (M);

  ml::Matrix twistRef (6, 6);

  for (unsigned i = 0; i < 6; ++i)
    for (unsigned j = 0; j < 6; ++j)
      twistRef (i, j) = (i == j) ? 1. : 0.;

  MATRIX_6x6_BOOST_REQUIRE_CLOSE (twist, twistRef, 0.001);
}

BOOST_AUTO_TEST_CASE (constructor_rotation_only)
{
  dynamicgraph::sot::MatrixHomogeneous M;

  MATRIX_4x4_INIT (M,
		   0.,  0.,  1., 0.,
		   1.,  0.,  0., 0.,
		   0., -1.,  0., 0.,
		   0.,  0.,  0., 1.);
  dynamicgraph::sot::MatrixTwist twist (M);


  ml::Matrix twistRef (6, 6);
  MATRIX_6x6_INIT (twistRef,
		   0.,  0.,  1., 0.,  0., 0.,
		   1.,  0.,  0., 0.,  0., 0.,
		   0., -1.,  0., 0.,  0., 0.,
		   0.,  0.,  0., 0.,  0., 1.,
		   0.,  0.,  0., 1.,  0., 0.,
		   0.,  0.,  0., 0., -1., 0.);

  MATRIX_6x6_BOOST_REQUIRE_CLOSE (twist, twistRef, 0.001);
}

BOOST_AUTO_TEST_CASE (constructor_translation_only)
{
  dynamicgraph::sot::MatrixHomogeneous M;

  MATRIX_4x4_INIT (M,
		   1., 0.,  0., 11.,
		   0., 1.,  0., 22.,
		   0., 0.,  1., 33.,
		   0., 0.,  0., 1.);
  dynamicgraph::sot::MatrixTwist twist (M);

  //FIXME: to be checked.
  ml::Matrix twistRef (6, 6);
  MATRIX_6x6_INIT (twistRef,
		   1., 0., 0.,   0., -33.,  22.,
		   0., 1., 0.,  33.,   0., -11.,
		   0., 0., 1., -22.,  11.,   0.,
		   0., 0., 0.,   1.,   0.,   0.,
		   0., 0., 0.,   0.,   1.,   0.,
		   0., 0., 0.,   0.,   0.,   1.);

  MATRIX_6x6_BOOST_REQUIRE_CLOSE (twist, twistRef, 0.001);
}

//FIXME: rotation+translation test case.
