// Copyright 2011 Florent Lamiraux

#include <sstream>

#define BOOST_TEST_MODULE matrix_homogeneous

#include <boost/math/constants/constants.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>
#include <sot/core/matrix-geometry.hh>

using boost::math::constants::pi;
using boost::test_tools::output_test_stream;

namespace dg = dynamicgraph;

#define MATRIX_BOOST_REQUIRE_CLOSE(N, M, LEFT, RIGHT, TOLERANCE)               \
  for (unsigned i = 0; i < N; ++i)                                             \
    for (unsigned j = 0; j < M; ++j)                                           \
  BOOST_REQUIRE_CLOSE(LEFT(i, j), RIGHT(i, j), TOLERANCE)

#define MATRIX_4x4_BOOST_REQUIRE_CLOSE(LEFT, RIGHT, TOLERANCE)                 \
  MATRIX_BOOST_REQUIRE_CLOSE(4, 4, LEFT, RIGHT, TOLERANCE)

#define MATRIX_IDENTITY_4x4_REQUIRE_CLOSE(M, TOLERANCE)                        \
  for (unsigned i = 0; i < 4; ++i)                                             \
    for (unsigned j = 0; j < 4; ++j)                                           \
      if (i == j)                                                              \
        BOOST_REQUIRE_CLOSE(M(i, j), 1., TOLERANCE);                           \
      else                                                                     \
        BOOST_CHECK_SMALL(M(i, j), .01 * TOLERANCE)

#define MATRIX_HOMO_INIT(M, tx, ty, tz, roll, pitch, yaw)                      \
  M(0, 0) = cos(pitch) * cos(yaw);                                             \
  M(0, 1) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);          \
  M(0, 2) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);          \
  M(0, 3) = tx;                                                                \
  M(1, 0) = cos(pitch) * sin(yaw);                                             \
  M(1, 1) = sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw);          \
  M(1, 2) = cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw);          \
  M(1, 3) = ty;                                                                \
  M(2, 0) = -sin(pitch);                                                       \
  M(2, 1) = sin(roll) * cos(pitch);                                            \
  M(2, 2) = cos(roll) * cos(pitch);                                            \
  M(2, 3) = tz;                                                                \
  M(3, 0) = 0.;                                                                \
  M(3, 1) = 0.;                                                                \
  M(3, 2) = 0.;                                                                \
  M(3, 3) = 1.

BOOST_AUTO_TEST_CASE(product) {
  for (unsigned int i = 0; i < 1000; i++) {
    double tx, ty, tz;
    double roll, pitch, yaw;
    dynamicgraph::sot::MatrixHomogeneous H1;
    tx = (10. * rand()) / RAND_MAX - 5.;
    ty = (10. * rand()) / RAND_MAX - 5.;
    tz = (10. * rand()) / RAND_MAX - 5.;
    roll = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    pitch = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    yaw = (2 * pi<double>() * rand()) / RAND_MAX - pi<double>();
    MATRIX_HOMO_INIT(H1, tx, ty, tz, roll, pitch, yaw);
    dg::Matrix M1(H1.matrix());
    dynamicgraph::sot::MatrixHomogeneous H2;
    tx = (10. * rand()) / RAND_MAX;
    ty = (10. * rand()) / RAND_MAX - 5.;
    tz = (10. * rand()) / RAND_MAX - 5.;
    roll = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    pitch = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    yaw = (2 * pi<double>() * rand()) / RAND_MAX - pi<double>();
    MATRIX_HOMO_INIT(H2, tx, ty, tz, roll, pitch, yaw);
    dg::Matrix M2(H2.matrix());
    dynamicgraph::sot::MatrixHomogeneous H3 = H1 * H2;
    dg::Matrix M3;
    M3 = M1 * M2;

    MATRIX_4x4_BOOST_REQUIRE_CLOSE(M3, H3.matrix(), 0.0001);
  }
}

BOOST_AUTO_TEST_CASE(inverse) {
  for (unsigned int i = 0; i < 1000; i++) {
    double tx, ty, tz;
    double roll, pitch, yaw;
    dynamicgraph::sot::MatrixHomogeneous H1;
    tx = (10. * rand()) / RAND_MAX - 5.;
    ty = (10. * rand()) / RAND_MAX - 5.;
    tz = (10. * rand()) / RAND_MAX - 5.;
    roll = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    pitch = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    yaw = (2 * pi<double>() * rand()) / RAND_MAX - pi<double>();
    MATRIX_HOMO_INIT(H1, tx, ty, tz, roll, pitch, yaw);
    dynamicgraph::sot::MatrixHomogeneous H2;
    tx = (10. * rand()) / RAND_MAX;
    ty = (10. * rand()) / RAND_MAX - 5.;
    tz = (10. * rand()) / RAND_MAX - 5.;
    roll = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    pitch = (pi<double>() * rand()) / RAND_MAX - .5 * pi<double>();
    yaw = (2 * pi<double>() * rand()) / RAND_MAX - pi<double>();
    MATRIX_HOMO_INIT(H2, tx, ty, tz, roll, pitch, yaw);
    dynamicgraph::sot::MatrixHomogeneous H3 = H1 * H2;
    dynamicgraph::sot::MatrixHomogeneous invH1, invH2, invH3;
    invH1 = H1.inverse();
    invH2 = H2.inverse();
    invH3 = H3.inverse();

    dynamicgraph::sot::MatrixHomogeneous I4;
    dynamicgraph::sot::MatrixHomogeneous P1 = H1 * invH1;
    dynamicgraph::sot::MatrixHomogeneous P2 = H2 * invH2;
    dynamicgraph::sot::MatrixHomogeneous P3 = H3 * invH3;
    dynamicgraph::sot::MatrixHomogeneous P4 = invH2 * invH1;

    MATRIX_IDENTITY_4x4_REQUIRE_CLOSE(P1, 0.0001);
    MATRIX_IDENTITY_4x4_REQUIRE_CLOSE(P2, 0.0001);
    MATRIX_IDENTITY_4x4_REQUIRE_CLOSE(P3, 0.0001);
    MATRIX_4x4_BOOST_REQUIRE_CLOSE(P4, invH3, 0.0001);
  }
}
