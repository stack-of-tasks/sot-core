#include <sot/core/device.hh>
#include <sot/core/flags.hh>
#include <sot/core/integrator.hh>

#include "dynamic-graph/python/module.hh"
#include "dynamic-graph/python/signal.hh"

namespace dg = dynamicgraph;
namespace dgs = dynamicgraph::sot;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");

  using dgs::PeriodicCall;
  bp::class_<PeriodicCall, boost::noncopyable>("PeriodicCall", bp::no_init)
      .def("addSignal",
           static_cast<void (PeriodicCall::*)(const std::string &,
                                              dg::SignalBase<int> &)>(
               &PeriodicCall::addSignal),
           "Add the signal to the refresh list", (bp::arg("name"), "signal"))
      .def("addSignal",
           static_cast<void (PeriodicCall::*)(const std::string &)>(
               &PeriodicCall::addSignal),
           "Add the signal to the refresh list", (bp::arg("signal_name")))

      .def("addDownsampledSignal",
           static_cast<void (PeriodicCall::*)(
               const std::string &, dg::SignalBase<int> &,
               const unsigned int &)>(&PeriodicCall::addDownsampledSignal),
           "Add the signal to the refresh list\n"
           "The downsampling factor: 1 means every time, "
           "2 means every other time, etc...",
           (bp::arg("name"), "signal", "factor"))
      .def("addDownsampledSignal",
           static_cast<void (PeriodicCall::*)(const std::string &,
                                              const unsigned int &)>(
               &PeriodicCall::addDownsampledSignal),
           "Add the signal to the refresh list\n"
           "The downsampling factor: 1 means every time, "
           "2 means every other time, etc...",
           (bp::arg("signal_name"), "factor"))

      .def("rmSignal", &PeriodicCall::rmSignal,
           "Remove the signal to the refresh list", bp::arg("name"))
      .def("clear", &PeriodicCall::clear,
           "Clear all signals and commands from the refresh list.")
      .def(
          "__str__", +[](const PeriodicCall &e) {
            std::ostringstream os;
            e.display(os);
            return os.str();
          });

  dynamicgraph::python::exposeEntity<dgs::Device>().def(
      "getControlSize", &dgs::Device::getControlSize,
      "Get number of joints controlled by the device.");

  using dgs::Flags;
  bp::class_<Flags>("Flags", bp::init<>())
      .def(bp::init<const char *>())
      .def("__init__", bp::make_constructor(+[](bp::list bools) {
             std::vector<bool> flags(bp::len(bools));
             for (std::size_t i = 0; i < flags.size(); ++i)
               flags[i] = bp::extract<bool>(bools[i]);
             return new Flags(flags);
           }))
      .def("__init__", bp::make_constructor(+[](bp::tuple bools) {
             std::vector<bool> flags(bp::len(bools));
             for (std::size_t i = 0; i < flags.size(); ++i)
               flags[i] = bp::extract<bool>(bools[i]);
             return new Flags(flags);
           }))
      .def("add", &Flags::add)
      .def("set", &Flags::set)
      .def("unset", &Flags::unset)

      .def(bp::self & bp::self)
      .def(bp::self | bp::self)
      .def(bp::self &= bp::self)
      .def(bp::self |= bp::self)

      .def("__call__", &Flags::operator())
      .def("__bool__", &Flags::operator bool)
      .def("reversed", &Flags::operator!)

      .def(
          "set",
          +[](Flags &f, const std::string &s) {
            std::istringstream is(s);
            is >> f;
          })
      .def(
          "__str__", +[](const Flags &f) {
            std::ostringstream os;
            os << f;
            return os.str();
          });

  dg::python::exposeSignalsOfType<Flags, int>("Flags");
  dg::python::exposeEntity<dgs::Integrator, bp::bases<dg::Entity>,
                           dg::python::AddCommands>()
      .add_property("after",
                    bp::make_function(&dgs::Integrator::periodicCallAfter,
                                      reference_existing_object()))
      .add_property("before",
                    bp::make_function(&dgs::Integrator::periodicCallBefore,
                                      reference_existing_object()))
      .add_property("model",
                    bp::make_function(&dgs::Integrator::getModel,
                                      reference_existing_object()),
                    bp::make_function(&dgs::Integrator::setModel))
      .def("setModel", &dgs::Integrator::setModel)
      .def("setInitialConfig", &dgs::Integrator::setInitialConfig);

  typedef dgs::internal::Signal S_t;
  bp::class_<S_t, bp::bases<dg::Signal<dg::Vector, int> >, boost::noncopyable>
      obj("SignalIntegratorVector", bp::init<std::string>());
  obj.add_property(
      "value",
      bp::make_function(&S_t::accessCopy,
                        bp::return_value_policy<bp::copy_const_reference>()),
      &S_t::setConstant,  // TODO check the setter
      "the signal value.\n"
      "warning: for Eigen objects, sig.value[0] = 1. may not work).");
}
