/** \page page_RequirementsInstallation Requirements and Installation

\section sec_requirements Requirements
This package should be compiled and installed with cmake.

This library is based on several packages:
 \li <c>dynamic-graph</c> (https://github.com/stack-of-tasks/dynamic-graph)
 \li <c>eigen</c>
 \li <c>boost</c>
 \li <c>pthread</c>
 \li <c>python</c>
 \li <c>pinocchio</c> (https://github.com/stack-of-tasks/pinocchio)

Their presence will be checked with the use of pkg-config.

\section sec_installation Installation
\subsection subsec_ins_binaries From binary package
The recommended way to install sot-core is to use
<a href="https://robotpkg.openrobots.org/install.html">robotpkg</a> and its
<a href="https://robotpkg.openrobots.org/robotpkg-wip.html">wip
extension</a>.

A short and quick way to get it on 16.04 LTS is to do the following:
\verbatim
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub kinetic
robotpkg deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub
kinetic robotpkg EOF \endverbatim This created a file to add robotpkg and
robotpkg/wip as apt repositories. To identify the packages from this repository
the server key must be added: \verbatim curl
http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
\endverbatim
The list of available packages is provided through:
\verbatim
sudo apt-get update
\endverbatim
Finally the package is installed with:
\verbatim
sudo apt-get install robotpkg-sot-core-v3
\endverbatim


\subsection subsec_ins_sourcefiles From source files
Although not recommended you can install the repository
after installing all the dependencies and type:
\verbatim
mkdir _build
cd _build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j 8
\endverbatim

To build the documentation
\verbatim
make doc
\endverbatim

To test the code
\verbatim
make test
\endverbatim


*/
