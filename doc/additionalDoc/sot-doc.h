/** \page page_sot Stack-of-Tasks
    
    \f$ {\bf e}(t)= {\bf s}^*(t) - {\bf s}(t) \f$
    thus:
    \f$
    \dot{\bf e}(t)= \dot{\bf s}^*(t) - \dot{\bf s}(t)
    \f$

    \f$ 
    \frac{\delta {\bf e}}{\delta {\bf t}}=
    \frac{\delta {\bf s}^*}{\delta {\bf t}} - 
    \frac{\delta {\bf s}}{\delta {\bf q}}
    \frac{\delta {\bf q}}{\delta {\bf t}}
    \f$

    \f$ 
    \dot{\bf e} =
    \dot{\bf s}^* - 
    {\bf J} \dot{\bf q}
    \f$

    \f$ 
    \Leftrightarrow \dot{\bf q}  =
    {\bf J}^{+}(\dot{\bf s}^* - \dot{\bf e})
    \f$

    Assuming that:
    
    \f$ 
    \dot{\bf e}^* = -\lambda {\bf e} = 
    - \lambda
    ({\bf s}^* - \hat{\bf s} )
    \f$
    with \f$ \hat{\bf s}\f$ is the measured feature.
    
    Imposing \f$\dot{\bf e} = \dot{\bf e}^* \f$
    then we have:
    
    \f$
    \dot{\bf q}  =
    {\bf J}^{+}(\dot{\bf s}^* + \lambda {\bf e}) = 
    {\bf J}^{+}(\dot{\bf s}^* + \lambda{\bf s}^* - \lambda \hat{\bf s})
    \f$
*/
