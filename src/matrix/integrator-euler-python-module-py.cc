#include <dynamic-graph/python/module.hh>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <dynamic-graph/python/dynamic-graph-py.hh>

#include <eigenpy/registration.hpp>

#include <sot/core/integrator-euler.hh>

namespace dg = dynamicgraph;
namespace dgc = dynamicgraph::command;
namespace dgs = dynamicgraph::sot;
using dg::Matrix;
using dg::Vector;

template <typename S, typename C> void exposeIntegratorEuler() {
  typedef dgs::IntegratorEuler<S, C> IE_t;

  const std::string cName = dgc::Value::typeName(dgc::ValueHelper<C>::TypeID);

  dg::python::exposeEntity<IE_t>()
      .add_property("numerators",
                    +[](const IE_t &e) {
                      return dg::python::to_py_list(e.numCoeffs().begin(),
                                                    e.numCoeffs().end());
                    },
                    +[](IE_t &e, bp::object iterable) {
                      e.numCoeffs(dg::python::to_std_vector<C>(iterable));
                    })
      .add_property("denominators",
                    +[](const IE_t &e) {
                      return dg::python::to_py_list(e.denomCoeffs().begin(),
                                                    e.denomCoeffs().end());
                    },
                    +[](IE_t &e, bp::object iterable) {
                      e.denomCoeffs(dg::python::to_std_vector<C>(iterable));
                    });
}

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");

  exposeIntegratorEuler<double, double>();
  exposeIntegratorEuler<Vector, double>();
  exposeIntegratorEuler<Vector, Matrix>();
}
