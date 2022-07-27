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

#include <sot/core/filter-differentiator.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#define BOOST_TEST_MODULE test - filter - differentiator

#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE(test_filter_differentiator) {
  sot::FilterDifferentiator *aFilterDiff =
      new FilterDifferentiator("filter_differentiator");

  Eigen::VectorXd filter_num(7), filter_den(7);

  filter_num(0) = 2.16439898e-05;
  filter_num(1) = 4.43473520e-05;
  filter_num(2) = -1.74065002e-05;
  filter_num(3) = -8.02197247e-05;
  filter_num(4) = -1.74065002e-05;
  filter_num(5) = 4.43473520e-05;
  filter_num(6) = 2.16439898e-05;

  filter_den(0) = 1.;
  filter_den(1) = -5.32595322;
  filter_den(2) = 11.89749109;
  filter_den(3) = -14.26803139;
  filter_den(4) = 9.68705647;
  filter_den(5) = -3.52968633;
  filter_den(6) = 0.53914042;

  double timestep = 0.001;
  int xSize = 16;
  aFilterDiff->init(timestep, xSize, filter_num, filter_den);

  srand(0);
  dynamicgraph::Vector aVec(16);
  for (unsigned int i = 0; i < 16; i++) aVec(i) = (double)(i + rand() % 100);
  aFilterDiff->m_xSIN = aVec;
  aFilterDiff->m_x_filteredSOUT.recompute(0);
  output_test_stream output;
  ostringstream anoss;
  dynamicgraph::Vector outVec;
  aFilterDiff->m_x_filteredSOUT.get(output);

  BOOST_CHECK(
      output.is_equal("82.5614 "
                      "86.5403 "
                      "78.5826 "
                      "17.9049 "
                      "96.4874 "
                      "39.7886 "
                      "91.5139 "
                      "98.4769 "
                      "56.6988 "
                      "29.8415 "
                      "71.6195 "
                      "37.7992 "
                      "101.461 "
                      "71.6195 "
                      "76.5931 "
                      "40.7834"));
}
