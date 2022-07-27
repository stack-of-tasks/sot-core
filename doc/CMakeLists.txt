#
#  Copyright 2010 CNRS
#  Author: Florent Lamiraux
#

# get the python script install path

IF(INSTALL_DOCUMENTATION)
  INSTALL (FILES
    pictures/feature.png
    pictures/task.png
    DESTINATION ${_PKG_CONFIG_DOXYGENDOCDIR}/pictures)
ENDIF(INSTALL_DOCUMENTATION)
