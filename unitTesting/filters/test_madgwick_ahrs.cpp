/*
 * Copyright 2019,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <iostream>
#include <sot/core/debug.hh>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <sot/core/madgwickahrs.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#define BOOST_TEST_MODULE test - filter - differentiator

#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE(test_filter_differentiator) {
  sot::MadgwickAHRS *aFilter = new MadgwickAHRS("MadgwickAHRS");

  double timestep = 0.001, beta = 0.01;
  aFilter->init(timestep);
  aFilter->set_beta(beta);

  srand(0);
  dynamicgraph::Vector acc(3);
  dynamicgraph::Vector angvel(3);
  acc(0) = 0.3;
  acc(1) = 0.2;
  acc(2) = 0.3;
  aFilter->m_accelerometerSIN = acc;
  angvel(0) = 0.1;
  angvel(1) = -0.1;
  angvel(2) = 0.3;
  aFilter->m_gyroscopeSIN = angvel;
  aFilter->m_imu_quatSOUT.recompute(0);
  output_test_stream output;
  ostringstream anoss;
  aFilter->m_imu_quatSOUT.get(output);
  aFilter->m_imu_quatSOUT.get(anoss);

  BOOST_CHECK(output.is_equal("           1\n"
                              "  5.5547e-05\n"
                              "-5.83205e-05\n"
                              "     0.00015\n"));
}
