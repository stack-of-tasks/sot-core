#include <sot/core/switch.hh>

namespace dg = ::dynamicgraph;
typedef boost::mpl::vector<
  dg::sot::Switch<bool, int>
  , dg::sot::Switch<dg::Vector, int>
  , dg::sot::Switch<dg::sot::MatrixHomogeneous, int>
> entities_t;
