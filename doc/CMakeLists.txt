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

SET(BUILD_DIR ${CMAKE_BINARY_DIR}/doc)

SET(DOCDIR share/doc/${PROJECT_NAME})
SET(HTMLDIR ${DOCDIR}/html)

# Add `doc' target.
ADD_CUSTOM_TARGET(doc ALL DEPENDS html/index.html)

# Generate Doxygen configuration file.
CONFIGURE_FILE(package.dox.cmake doc/package.dox)

# Generating the Doxygen documentation
ADD_CUSTOM_COMMAND(
  OUTPUT html/index.html
  COMMAND ${DOXYGEN_EXECUTABLE} doc/package.dox
  COMMENT Generating Doxygen documentation
)

# Doxygen documentation installation rules.
INSTALL(DIRECTORY ${BUILD_DIR}/html DESTINATION ${DOCDIR})
INSTALL(FILES ${BUILD_DIR}/${PROJECT_NAME}.doxytag DESTINATION ${HTMLDIR})
INSTALL(DIRECTORY pictures DESTINATION ${HTMLDIR})
