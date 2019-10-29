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
  std::cout << "anoss:" << anoss << std::endl;
  BOOST_CHECK(output.is_equal("82.5614\n"
                              "86.5403\n"
                              "78.5826\n"
                              "17.9049\n"
                              "96.4874\n"
                              "39.7886\n"
                              "91.5139\n"
                              "98.4769\n"
                              "56.6988\n"
                              "29.8415\n"
                              "71.6195\n"
                              "37.7992\n"
                              "101.461\n"
                              "71.6195\n"
                              "76.5931\n"
                              "40.7834\n"));
}
