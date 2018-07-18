#
#  Copyright 2010 CNRS
#  Author: Florent Lamiraux
#

# get the python script install path

IF(BUILD_PYTHON_INTERFACE)
  INCLUDE(../cmake/sphinx.cmake)
  SPHINX_SETUP()
  SPHINX_FINALIZE()
  INSTALL (FILES
    pictures/feature.png
    pictures/task.png
    DESTINATION ${_PKG_CONFIG_DOXYGENDOCDIR}/pictures)
ENDIF(BUILD_PYTHON_INTERFACE)
