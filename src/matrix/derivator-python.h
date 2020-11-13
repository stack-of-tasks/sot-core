#include <sot/core/derivator.hh>

namespace dg = ::dynamicgraph;
typedef boost::mpl::vector<dg::sot::Derivator<double>,
                           dg::sot::Derivator<dg::Vector>,
                           dg::sot::Derivator<dg::Matrix>,
                           dg::sot::Derivator<dg::sot::VectorQuaternion> >
    entities_t;
