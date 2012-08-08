#
#  Copyright 2010 CNRS
#  Author: Florent Lamiraux
#

INCLUDE(../cmake/sphinx.cmake)

SPHINX_SETUP()
SPHINX_FINALIZE()
INSTALL (FILES
  pictures/feature.png 
  pictures/task.png 
  DESTINATION ${PKG_CONFIG_DOXYGENDOCDIR}/pictures)