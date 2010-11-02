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
you may choose to include (load) in your program only the functionallity you
need.

Aside from the entities, there is a code base to the library, libsot-core,
that provides functions and code common to all modules. All plugins
developed here link with libsot-core. For example, common mathematical
entities, definitions and functions are in that library's base.

\section subsec_Tasks Tasks
They are a certain number of pre-written tasks that can be used.

\section sec_RequirementsInstallation Requirements and Installation
 The library assumes that Boost is installed.

\section subsec_tools Tools

\section References
\anchor Mansard2007
<b>"Task sequencing for sensor-based control"</b>,
<em>N. Mansard, F. Chaumette,</em>
IEEE Trans. on Robotics, 23(1):60-72, February 2007

@defgroup Installation

<h1> Configuration and compiling with cmake (recommended) </h1>
In the repository create a build directory and go inside :<br>
$ make build <br>
$ cd build <br>

For configuration try:<br>
$ ccmake ..<br>
Press [c] to configure and start the autodetection.<br>
The system will try to detect as much as possible the needed
information but it might not work depending on your installation.
If everything worked,
press [g] to generate all the necessary makefiles. <br>
Then type <br>
$ make <br>
to compile the Stack Of Tasks. <br>
Finally to install it properly type:<br>
$ make install <br>
If cmake fails to detect some of the needed tools please read the following.

<h2> Installation prefix </h2>
To specify the installation directory you have to set CMAKE_INSTALL_PREFIX. <br>

<h2> Boost thread library </h2>
First check that the BOOST_THREAD_LIB_NAME is the one you have in your
boost installation directory. To find the one on your system type: <br>
$ ls -al /usr/lib/libboost*thread*  <br>
and if the following line is displayed: <br>
lrwxrwxrwx 1 root root    50 2008-08-28 09:00 /usr/lib/libboost_thread-gcc41-mt-1_34_1.so -> /usr/lib/libboost_thread-gcc41-mt-1_34_1.so.1.34.1 <br>
You can put: <br>
boost_thread-gcc41-mt-1_34_1<br>
<br>
<h2> OpenHRP </h2>
The system looks for OpenHRP in the <b>${OPENHRPHOME}</b> environment variable,
and in /home/user_name/src/OpenHRP otherwise. CMake sets an internal variable
called <b>OPENHRP_HOME</b>. If version 3 is installed (detected by the presence
of the file Make.vars.ubuntu.8.04) the internal variable called <b>OPENHRP3_HOME</b>.


<h1> Configuration and compiling with automake </h1>
Type the following command: <br>
$ aclocal -I macros && automake -a <br>
$ autoconf  <br>
Usually you should type <br>
$ configure <br>
but if you really want your system to be properly configured with OpenHRP: <br>
$ ./configure  CXX=g++-3.3 --with-boost-thread=/usr/local/3.3 --with-dynamicsjrl=/usr/local/ --with-maal --with-openhrp=/path/to/OpenHRP/ CXXFLAGS="-DNDEBUG -O2" <br>

$ make install <br>
Once this is done you have to remove the plugin compiled with the wrong image of libsot-0.so: <br>
$ rm ./lib/plugin/sot*.so <br>
$ make <br>

To create the tests.<br>
$ make tests -k <br>

Do not take care to the warnings coming from aclocal or automake executions
(they are only warnings)


@defgroup stackoftasks Stack of Tasks
This part implements the general framework of the stack
of stack i.e. its recursive definition

@defgroup tasks Tasks
@ingroup stackoftasks

Each task implements a specific controller which
can be a free space task, visual servoing, a constraint,
and so on ...

@defgroup features Features
Each feature compute some information
from measurements, from the robot state,or/and
from others features. Features should be lightweight
regarding computation and be included at this level
only if necessary.

@defgroup softwaresupport Plugin management modules.

@defgroup factory Factory
@ingroup softwaresupport

This code implements the factory design pattern, making creation of features,
tasks and objects available.

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


@defgroup signals Signals
@ingroup softwaresupport
This part provides the mechanism to transfer information
from one feature to another.


@defgroup codesourceexamples Code source examples
Various examples to use the different parts of the Stack of Tasks.
They are all located in the tests directory.


*/

