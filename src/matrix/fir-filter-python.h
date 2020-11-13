#include <sot/core/fir-filter.hh>

namespace dg = ::dynamicgraph;
typedef boost::mpl::vector<dg::sot::FIRFilter<double, double>,
                           dg::sot::FIRFilter<dg::Vector, double>,
                           dg::sot::FIRFilter<dg::Vector, dg::Matrix> >
    entities_t;
