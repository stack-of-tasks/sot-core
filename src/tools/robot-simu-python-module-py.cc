#include <boost/python.hpp>
#include "dynamic-graph/python/module.hh"
#include <dynamic-graph/python/signal.hh>
#include <sot/core/robot-simu.hh>
#include <sot/core/flags.hh>

namespace dg = dynamicgraph;
namespace dgs = dynamicgraph::sot;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph.sot.core.wrap");
  dynamicgraph::python::exposeEntity<dgs::RobotSimu,
				     bp::bases<dg::sot::Device> >();
}
