#include <sot/core/integrator-euler.hh>

namespace dg = dynamicgraph;
using dg::Vector;
using dg::Matrix;

typedef boost::mpl::vector<
  dg::sot::IntegratorEuler<double, double>
, dg::sot::IntegratorEuler<Vector, double>
, dg::sot::IntegratorEuler<Vector, Matrix>
> entities_t;
