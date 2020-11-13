#include <sot/core/feature-pose.hh>

namespace dgs = dynamicgraph::sot;

typedef boost::mpl::vector<dgs::FeaturePose<dgs::SE3Representation>,
                           dgs::FeaturePose<dgs::R3xSO3Representation> >
    entities_t;
