#
# Copyright 2010 CNRS Author: Florent Lamiraux
#

# get the python script install path

if(INSTALL_DOCUMENTATION)
  install(FILES pictures/feature.png pictures/task.png
          DESTINATION ${_PKG_CONFIG_DOXYGENDOCDIR}/pictures)
endif(INSTALL_DOCUMENTATION)
