/** \mainpage
\section sec_intro Introduction

This library implements a modular architecture to test and experiment
controllers in the Stack of Tasks Framework as defined in \ref Mansard2007.
It is specifically targeted to retain real-time performance while
having high level software capabilities, like plugins and scripts.

\section sec_organization Organization of the code

The code is based on the dynamic-graph package, which provides the
framework from which sot-core relies. Hence most of the code in sot-core
consists of classes that derive from entities. These entities are usually
compiled and linked in their own dynamic library, as a "plugin"; hence
you may choose to include (load) in your program only the ones you
need.

Aside from the entities, there is a code base to the library, libsot-core,
that provides functions and code common to all modules. All plugins
developed here link with libsot-core. For example, common mathematical
entities, definitions and functions are in that library's base.
See \ref sot_core_base for a list of what's in this code base.

This library implements
\li A solver
\li The framework of Tasks and Features, as defined in \ref Mansard2007, that
\li Base classes for working with tasks and features, such as special matrices
(rotation, twist) and vectors (quaternions, etc.)
\li A basic C++ "machinery"

\section using Using the Stack of Tasks
Using dynamic-graph objects (pool and factory), either through C++ code
or the built-in interpreter, we can
For people who develop software on the HRP2 robot, this package
alone is insufficient to simulate and control a robot.

The following packages are recommended* in that case:
\li sot-dynamic
\li sot-pattern-generator
\li sot-openhrp (openhrp plugin)
\li sot-openhrp-scripts

* These packages are in development at the time of writing and may
be separated or joined along the way. Please check their status and updates
at the jrl-umi3218 web page: https://github.com/jrl-umi3218

\section sec_RequirementsInstallation Requirements and Installation

This package should be compiled and installed with cmake.

This library is based on several packages:
 \li dynamic-graph (https://github.com/jrl-umi3218/dynamic-graph)
 \li MatrixAbstractLayer (https://github.com/jrl-umi3218/jrl-mal)
 \li Lapack
 \li Boost
 \li pthread

Their presence will be checked with the use of pkg-config.

There are talks of open-sourcing the library on github. When this
is done, installation instructions will be on the project's github
page.

\section sot_base Base library
This package is centered around a base library that implements
the basic classes needed for operation of the stack of tasks. For
more information, see \ref sot_core_base.

\section sot_plugins Plugins
While the main library provides a basic framework for computation of a
control law using the Stack of Tasks, it is not expressive enough for
typical usage scenarios (for example, controlling the humanoid robot
HRP-2). Hence, several "specialized" features and tasks have been developed,
and can be used in the Stack of Tasks. For a list of plugins and a short
description, see \ref plugins_list.

\section operation Operation of the stack of tasks

\subsection subsec_intro Introduction
As explained in \ref Mansard2007, tasks can be pushed on the stack
of tasks (SoT), and thus assigned a priority. The SoT then calculates
(in an iterative manner) the control law, or joint velocity, necessary
to realize the tasks (usually, bring a feature to its desired value).

\subsection tasks Tasks

Each task implements a specific controller which
can be a free space task, visual servoing, a constraint,
and so on ... a task is mainly defined by the feature it is
handling and its gain.

After being defined, tasks are pushed in the Stack of Tasks which then
calculates a control law as explained in \ref Mansard2007.

\subsection features Features
Each feature computes some information
from measurements, from the robot state,or/and
from others features. Features should be lightweight
regarding computation and be included at this level
only if necessary.

\section References
\anchor Mansard2007
<b>"Task sequencing for sensor-based control"</b>,
<em>N. Mansard, F. Chaumette,</em>
IEEE Trans. on Robotics, 23(1):60-72, February 2007





@defgroup sot_core_base Sot-core base library (libsot-core.[dll|so])
For developers, this is the base library that your project has to
be linked with. It defines the related algorithms and entities
using the dynamic-graph framework.

\section What's in the box?

All the C++ code is placed in the namespace "sot". In the base
library, the following classes are present:

\subsection subsec_sot Stack of Task solvers
The class sot::Sot is the class where tasks can be pushed, moved up
and down, and removed on the stack.

\subsection subsec_exceptions Exceptions
The library defines the following classes of exceptions that derive from std::exception:
\li sot::ExceptionAbstract (the base class for all exceptions in sot-core; use it to
catch only sot-core exceptions)
\li sot::ExceptionDynamic
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

\subsection subsec_Features Features
The class sot::FeatureAbstract is the base class for features.
all other classes are in entity \ref sot_plugins "plugins".
For more information on what is a feature, see \ref features.

\subsection subsec_Tasks Tasks
They are a certain number of pre-written tasks that can be used.
They all derive from the task sot::TaskAbstract; specific tasks
are defined as \ref sot_plugins "plugins".

\subsection subsec_tools Mathematical base
The following classes encapsulate common mathematical objects, and
are all defined in the base library; see each individual class documentation
for reference.
\li Vectors: (sot::VectorQuaternion, sot::VectorRollPitchYaw, sot::VectorUTheta)
\li Matrices: (sot::MatrixForce, sot::MatrixHomogeneous, sot::MatrixRotation, sot::MatrixTwist)
\li sot::MultiBound can be used to enforce bounds on numeric values

\subsection subsec_others Others
The base library also contains functions for adaptation and extension of
the dynamic-graph shell; see additional-functions.cpp, factory.cpp and pool.cpp
directly for these. In a nutshell, a typical user shouldn't need these.
See \ref factory for additional information.



\defgroup plugins_list List of plugins
These plugins are linked with the base library.


@defgroup factory Factory

This code implements the factory design pattern, making creation of features,
tasks and other objects available.

Objects, which are derived from Entities, Tasks, or Features, can be
 declared within the code and compiled to shared librairies (.so/.dll files).
These librairies can be loaded at run-time using the sotPluginLoader methods,
and at the same time register their class names to the Factory (see the
sotFactory documentation to learn how).

The Factory can then create instances of these objects and subsequently
register them in the Pool, where they can be listed, accessed, and acted upon
(see sotPoolStorage documentation). Basic commands defined by entities include
signal connection graph file generation, help and name print, and signals.

Finally, a shell (command-line) interface is made available thanks to the
sotInterpretor class (see the file test_shell.cpp). Objects deriving from
Entity can expose their own commands by overriding the Entity's default
commandLine() method. It is possible to load a plugin to register custom
shell commands; see sotShellFunctions and sotShellProcedure for an example.

The public static objects (singletons) made available by including the
corresponding headers in this module are:
\li sotFactory: sotFactoryStorage
\li sotPool: sotPoolStorage

\image html schema_plugin.png


@defgroup codesourceexamples Code source examples
Various examples to use the different parts of the Stack of Tasks.
They are all located in the tests directory.


*/

