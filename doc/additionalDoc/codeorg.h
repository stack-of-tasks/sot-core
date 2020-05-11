/** \page codeorganization Code organization

    \section sec_organization Organization of the code

The code is based on the <c>dynamic-graph</c> package, which provides the
framework on which sot-core relies. Hence most of the code in <c>sot-core</c>
consists of classes that derive from dynamic_graph::Entity.
These entities are usually
compiled and linked in their own dynamic library, as a python module

Aside from the python modules, this package installs a core library libsot-core,
that provides functions and code common to all modules. All python modules
developed here link with libsot-core. For example, common mathematical
entities, definitions and functions are in the core library.
See \ref sot_core_base for a list of what's in this code base.

This library implements
\li A solver for the control law using tasks and features: the stack of tasks
\li The framework of Tasks and Features, as defined in \cite Mansard2007, that
\li Base classes for working with tasks and features, such as special matrices
(rotation, twist) and vectors (quaternions, etc.)
\li A basic C++ "machinery", the \ref factory, that "makes things work"

*/
