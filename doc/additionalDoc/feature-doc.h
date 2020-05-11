/* Olivier Stasse
 * CNRS, LAAS 2019
 */
/** \page page_featuredoc Features

    The general definition of feature is provided by
    dynamicgraph::sot::FeatureAbstract .

    The list of features that are provided are:
    <ul>
    <li> Feature1D : \f$ L_2 \f$  norm of \f$ e(t)\f$
    dynamicgraph::sot::Feature1D</li>
    <li> FeatureGeneric : Externalisation of the Feature computation
    dynamicgraph::sot::FeatureGeneric </li>
    <li> FeatureJointLimits : Distance to the joint limits and its Jacobian
    dynamicgraph::sot::FeatureJointLimits </li>
    <li> FeatureLineDistance : Regulate the distance of a line through a body
    to a reference point
    dynamicgraph::sot::FeatureLineDistance </li>
    <li> FeaturePoint6d : Regulate the distance from a pose on a body to
    a point reference.
    dynamicgraph::sot::FeaturePoint6d </li>
    <li> FeaturePoint6dRelative : Regulate a relative distance between
    two bodies.
    dynamicgraph::sot::FeatuePoint6dRelative </li>
    <li> FeaturePosture : Regulate the posture between the current one
    and a reference.
    dynamicgraph::sot::FeaturePosture </li>
    <li> FeatureReferenceHelper : dynamicgraph::sot::FeatureReferenceHelper
    </li>
    <li> FeatureTask : dynamicgraph::sot::FeatureTask </li>
    <li> FeatureVector3 : dynamicgraph::sot::FeatureVector3 </li>
    <li> FeatureVisualPoint : : dynamicgraph::sot::FeatureVisualPoint </li>
    </ul>
 */
