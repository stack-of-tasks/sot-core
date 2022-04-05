/*
 * Copyright 2019
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <Eigen/LU>
#include <boost/mpl/if.hpp>
#include <boost/type_traits/is_same.hpp>
#include <pinocchio/multibody/liegroup/liegroup.hpp>
#include <sot/core/factory.hh>
#include <sot/core/feature-pose.hh>
#include <sot/core/feature-pose.hxx>
#include <sot/core/macros.hh>

using namespace dynamicgraph::sot;

SOT_CORE_DISABLE_WARNING_PUSH
SOT_CORE_DISABLE_WARNING_DEPRECATED
typedef FeaturePose<R3xSO3Representation> FeaturePose_t;
typedef FeaturePose<SE3Representation> FeaturePoseSE3_t;
template <>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePose_t, "FeaturePose");
template <>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePoseSE3_t, "FeaturePoseSE3");
SOT_CORE_DISABLE_WARNING_POP

namespace dynamicgraph {
namespace sot {
template class FeaturePose<R3xSO3Representation>;
template class FeaturePose<SE3Representation>;
}  // namespace sot
}  // namespace dynamicgraph
