#include "dynamic-graph/python/module.hh"
#include <sot/core/switch.hh>

namespace dg = dynamicgraph;
typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

template <typename T, typename Time> void exposeSwitch() {
  typedef dg::sot::Switch<T, Time> E_t;
  typedef typename E_t::Base B_t;
  dg::python::exposeEntity<E_t, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .def_readonly("sout", &E_t::SOUT)
      .def("sin", &B_t::getSignalIn, reference_existing_object())
      .add_property("n_sin", &B_t::getSignalNumber, &B_t::setSignalNumber,
                    "the number of input signal.")
      .def_readonly("selection", &E_t::selectionSIN)
      .def_readonly("boolSelection", &E_t::boolSelectionSIN)

      .def("setSignalNumber", &B_t::setSignalNumber,
           "set the number of input signal.", bp::arg("size"))
      .def("getSignalNumber", &B_t::getSignalNumber,
           "get the number of input signal.", bp::arg("size"));
}

BOOST_PYTHON_MODULE(wrap) {
  exposeSwitch<bool, int>();
  exposeSwitch<dg::Vector, int>();
  exposeSwitch<dg::sot::MatrixHomogeneous, int>();
}
