# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-core.
# sot-core is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-core is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-core. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-core)
SET(PROJECT_DESCRIPTION "Hierarchical task solver plug-in for dynamic-graph.")
SET(PROJECT_URL "http://github.com/jrl-umi3218/sot-core")

SET(CUSTOM_HEADER_DIR "sot/core")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

PKG_CONFIG_APPEND_LIBS("sot-core")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS thread filesystem program_options unit_test_framework system regex )
SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")

ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)
ADD_SUBDIRECTORY(doc)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
