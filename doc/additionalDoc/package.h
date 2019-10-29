/** \mainpage
\section sec_intro Introduction

This library implements a modular architecture to test and experiment
controllers in the Stack of Tasks Framework as defined in \cite Mansard2007.
It is specifically targeted to retain real-time performance while
having high level software capabilities, like plugins and python scripting.

The package includes the following functionnalities:
<ul>
<li> A parameter server object to register and collect information on your
robot (see \subpage page_parameter_server) </li>
<li> From the internal SoT point of view a class called Device which is
allowing a direct interaction with your robot.</li>
<li> From the extern SoT point of view a class which makes the connection
either with the hardware or with a simulator </li>
<li> A full kinematic engine implemented in an incremental manner.</li>
</ul>

\subpage page_RequirementsInstallation

\subpage page_featuredoc

\subpage page_sot

\subsection tasks Tasks

Each task implements a specific controller which
can be a free space task, visual servoing, a constraint,
and so on ... a task is mainly defined by the feature it is
handling and its gain.

After being defined, tasks are pushed in the Stack of Tasks which then
calculates a control law as explained in \cite Mansard2007.

See documentation of class dynamicgraph::sot::TaskAbstract for more information.



@defgroup sot_core_base Sot-core library (libsot-core.[dll|so])
For developers, this is the core library that your project has to
be linked with. It defines the related algorithms and entities
using the dynamic-graph framework.

\section presento What's in the box?

All the C++ code is placed in the namespace "dynamic_graph::sot". In the core
library, the following classes are present:

\subsection subsec_sot Stack of Task solvers
A hierarchical inequality solver, sot::SolverHierarchicalInequalities, is
present. For the maths, see \cite Mansard2007.

\subsection subsec_exceptions Exceptions
The library defines the following classes of exceptions that derive from
std::exception: \li sot::ExceptionAbstract (the base class for all exceptions in
sot-core; use it to catch only sot-core exceptions) \li sot::ExceptionDynamic
\li sot::ExceptionFactory (raised if issues in instancing tasks and features)
\li sot::ExceptionFeature (raised by a feature - see \ref subsec_Features)
\li sot::ExceptionSignal
\li sot::ExceptionTask (raised by a task - see \ref subsec_Tasks)
\li sot::ExceptionTools

\subsection subsec_signalcast Signal cast helpers
Using the dynamic-graph module's signal_disp<T>, signal_cast<T> or
signal_trace<T> methods, involves a cast to or from a
stringstream object (i.e. for all practical purposes, a string). For the
objects defined in sot-core (T = sot::*), most of these casts are defined.
Hence you can display sot::VectorQuaternion or sot::MatrixTwist objects as
strings, or set them from the command line (using a script interpreter from
the dynamic-graph package).


\subsection subsec_Tasks Tasks

Tasks are the hierarchical element of the stack of tasks.

A task computes a value and a Jacobian as output signals.
Once stacked into a solver, the solver will compute the control vector
that makes the task value converge toward zero in the
order defined by the priority levels. For more information see the documentation
of class dynamicgraph::sot:TaskAbstract.

\subsection subsec_tools Mathematical base
The following classes encapsulate common mathematical objects, and
are all defined in the core library; see each individual class documentation
for reference.
\li Vectors: (sot::VectorQuaternion, sot::VectorRollPitchYaw, sot::VectorUTheta)
\li Matrices: (sot::MatrixForce, sot::MatrixHomogeneous, sot::MatrixRotation,
sot::MatrixTwist) \li sot::MultiBound can be used to enforce bounds on numeric
values

\subsection subsec_others Others
The core library also contains functions for adaptation and extension of
the dynamic-graph shell; see additional-functions.cpp, factory.cpp and pool.cpp
directly for these. In a nutshell, a typical user shouldn't need these.
See \ref factory for additional information.



\defgroup plugins_list List of python modules
These python modules are linked with the core library.

        sot/sot-qr
        sot/weighted-sot
        sot/sot-h
        sot/sot

        math/op-point-modifier

        matrix/binary-op
        matrix/derivator
        matrix/fir-filter
        matrix/integrator-abstract
        matrix/integrator-euler
        matrix/matrix-constant
        matrix/unary-op
        matrix/vector-constant
        matrix/vector-to-rotation

        task/gain-adaptive
        task/task-pd
        task/constraint
        task/gain-hyperbolic
        task/task
        task/task-conti
        task/task-unilateral

        feature/feature-point6d
        feature/feature-vector3
        feature/feature-generic
        feature/feature-joint-limits
        feature/feature-1d
        feature/feature-point6d-relative
        feature/feature-visual-point
        feature/feature-task
        feature/feature-line-distance

        traces/reader

        tools/time-stamp
        tools/timer
        tools/seq-play
        tools/sequencer
        tools/robot-simu
        tools/periodic-call-entity
        tools/motion-period
        tools/neck-limitation
        tools/mailbox-vector
        tools/kalman
        tools/joint-limitator
        tools/gripper-control
        tools/com-freezer
        tools/clamp-workspace
        tools/binary-int-to-uint

        control/control-gr
        control/control-pd

@defgroup factory Factory

This code implements the factory design pattern, making creation of features,
tasks and other objects available.

Objects, which are derived from Entities, Tasks, or Features, can be
 declared within the code and compiled to C++ python modules.  These
 modules can be imported at run-time and register their class names to
 the Factory (see the sotFactory documentation to learn how).

The Factory can then create instances of these objects and subsequently
register them in the Pool, where they can be listed, accessed, and acted upon
(see sot::PoolStorage documentation). Basic commands defined by entities include
signal connection graph file generation, help and name print, and signals.

The public static objects (singletons) made available by including the
corresponding headers in this module are:
\li sot::PoolStorage, accessed by method getInstance.

\image html schema_plugin.png


@defgroup codesourceexamples Code source examples
Various examples to use the different parts of the Stack of Tasks.
They are all located in the tests directory.


*/
