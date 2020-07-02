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
#include <boost/mpl/if.hpp>
#include <boost/type_traits/is_same.hpp>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <pinocchio/multibody/liegroup/liegroup.hpp>

#include <Eigen/LU>

#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/feature-pose.hh>
#include <sot/core/feature-pose.hxx>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

typedef pinocchio::CartesianProductOperation<
    pinocchio::VectorSpaceOperationTpl<3, double>,
    pinocchio::SpecialOrthogonalOperationTpl<3, double> >
    R3xSO3_t;
typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3_t;

template <Representation_t representation> struct LG_t {
  typedef typename boost::mpl::if_c<representation == SE3Representation, SE3_t,
                                    R3xSO3_t>::type type;
};


typedef FeaturePose<R3xSO3Representation> FeaturePose_t;
typedef FeaturePose<SE3Representation> FeaturePoseSE3_t;
template <> DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePose_t, "FeaturePose");
template <>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePoseSE3_t, "FeaturePoseSE3");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

static const MatrixHomogeneous Id(MatrixHomogeneous::Identity());

