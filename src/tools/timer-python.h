#include <sot/core/timer.hh>
#include <sot/core/matrix-geometry.hh>

typedef boost::mpl::vector<
  Timer<dynamicgraph::Vector>
, Timer<dynamicgraph::Matrix>
, Timer<dynamicgraph::sot::MatrixHomogeneous>
, Timer<double>
> entities_t;
