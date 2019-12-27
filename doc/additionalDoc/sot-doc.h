/** \page page_sot Stack-of-Tasks

    As explained in class dynamicgraph::sot::FeatureAbstract,
    \f[
    {\bf E}(t) = {\bf e}({\bf q}(t), t)= {\bf s}({\bf q}(t)) - {\bf s}^*(t)
    \f]
    thus:
    \f[
    \dot{\bf E}= \frac{\partial{\bf e}}{\partial{\bf q}} \dot{\bf q}
    + \frac{\partial{\bf e}}{\partial t} \\
    \f]

    The features are responsible for computing:
    \li \f$ {\bf E}(t) \f$
    \li \f$ \frac{\partial{\bf e}}{\partial t} \f$
    \li \f$ \frac{\partial{\bf e}}{\partial{\bf q}} \f$


    The class dynamicgraph::sot::Task takes some features outputs as inputs.
    It imposes an exponential decrease, i.e.
    \f$ \dot{\bf E} = -\lambda {\bf E} \f$.
    It gives:
    \f[
    -\lambda {\bf e} =
    \frac{\partial{\bf e}}{\partial{\bf q}} \dot{\bf q}
    + \frac{\partial{\bf e}}{\partial t}
    \f]
    and computes solutions as:
    \f[
    \dot{\bf q} = \frac{\partial{\bf e}}{\partial{\bf q}}^{\dagger} \left(
        - \lambda {\bf e} - \frac{\partial{\bf e}}{\partial t}
    \right) + K v
    \f]

    where:
    \li \f$\frac{\partial{\bf e}}{\partial{\bf q}}^{\dagger}\f$ is the
        pseudo-inverse of \f$\frac{\partial{\bf e}}{\partial{\bf q}}\f$,
    \li \f$\dot{\bf q} \in \mathbb{R}^n\f$,
    \li \f$K \in \mathcal{M}_{n,m}(\mathbb{R})\f$ is a base of the kernel of
        \f$\frac{\partial{\bf e}}{\partial{\bf q}}\f$, of dimension \f$m\f$,
    \li and \f$v\f$ spans \f$\mathbb{R}^m\f$ so that \f$K v\f$ spans
        the kernel of
        \f$\frac{\partial{\bf e}}{\partial{\bf q}}\f$.

    \f$v\f$ is a free parameters left to tasks of lower priority.
*/
