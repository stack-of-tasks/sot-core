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

<<<<<<< HEAD
SET(${PROJECT_NAME}_HEADERS
#debug
contiifstream.h
debug.h

#exception
exception-abstract.h
exception-dynamic.h
exception-factory.h
exception-feature.h
exception-signal.h
exception-task.h
exception-tools.h

#signal

#matrix
binary-op.h
derivator.h
fir-filter.h
integrator-abstract.h
integrator-euler.h
matrix-constant.h
unary-op.h
vector-constant.h
vector-to-rotation.h

#factory
additional-functions.h
factory.h
macros-signal.h
pool.h

#math
matrix-force.h
matrix-twist.h
op-point-modifier.h
vector-quaternion.h
vector-roll-pitch-yaw.h

#feature
feature-point6d.h
feature-vector3.h
feature-abstract.h
feature-generic.h
feature-joint-limits.h
feature-1d.h
feature-point6d-relative.h
feature-visual-point.h
feature-task.h
feature-line-distance.h

#task
task-abstract.h
task-unilateral.h
task-pd.h
task-conti.h
multi-bound.h
constraint.h
gain-adaptive.h
task.h
gain-hyperbolic.h

#sot
flags.h
sot-qr.h
memory-task-sot.h
sot-h.h
sot.h
rotation-simple.h
weighted-sot.h
solver-hierarchical-inequalities.h

#traces
reader.h

#tools
utils-windows.h
time-stamp.h
timer.h
neck-limitation.h
motion-period.h
binary-int-to-uint.h
clamp-workspace.h
com-freezer.h
gripper-control.h
joint-limitator.h
kalman.h

#these need the boost thread library - reminder
mailbox.hxx
mailbox.t.cpp
mailbox-vector.h
)

# Recreate correct path for the headers
#--------------------------------------

SET(fullpath_${PROJECT_NAME}_HEADERS)
FOREACH(lHeader ${${PROJECT_NAME}_HEADERS})
  SET(fullpath_${PROJECT_NAME}_HEADERS 
    ${fullpath_${PROJECT_NAME}_HEADERS}
    ./${PROJECT_NAME}/${lHeader}
  )

ENDFOREACH(lHeader)

#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${fullpath_${PROJECT_NAME}_HEADERS}
	DESTINATION include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

=======
>>>>>>> Move headers in include/sot/core and rename .h into .hh.
SET(NEWHEADERS 
  sot/core/api.hh
  sot/core/device.hh
  sot/core/robot-simu.hh
  sot/core/matrix-homogeneous.hh
  sot/core/matrix-rotation.hh
  sot/core/vector-utheta.hh
  sot/core/vector-rotation.hh
  sot/core/contiifstream.hh
  sot/core/debug.hh
  sot/core/exception-abstract.hh
  sot/core/exception-dynamic.hh
  sot/core/exception-factory.hh
  sot/core/exception-feature.hh
  sot/core/exception-signal.hh
  sot/core/exception-task.hh
  sot/core/exception-tools.hh
  sot/core/binary-op.hh
  sot/core/derivator.hh
  sot/core/fir-filter.hh
  sot/core/integrator-abstract.hh
  sot/core/integrator-euler.hh
  sot/core/matrix-constant.hh
  sot/core/unary-op.hh
  sot/core/vector-constant.hh
  sot/core/vector-to-rotation.hh
  sot/core/additional-functions.hh
  sot/core/factory.hh
  sot/core/macros-signal.hh
  sot/core/pool.hh
  sot/core/matrix-force.hh
  sot/core/matrix-twist.hh
  sot/core/op-point-modifier.hh
  sot/core/vector-quaternion.hh
  sot/core/vector-roll-pitch-yaw.hh
  sot/core/feature-point6d.hh
  sot/core/feature-vector3.hh
  sot/core/feature-abstract.hh
  sot/core/feature-generic.hh
  sot/core/feature-joint-limits.hh
  sot/core/feature-1d.hh
  sot/core/feature-point6d-relative.hh
  sot/core/feature-visual-point.hh
  sot/core/feature-task.hh
  sot/core/feature-line-distance.hh
  sot/core/task-abstract.hh
  sot/core/task-unilateral.hh
  sot/core/task-pd.hh
  sot/core/task-conti.hh
  sot/core/multi-bound.hh
  sot/core/constraint.hh
  sot/core/gain-adaptive.hh
  sot/core/task.hh
  sot/core/gain-hyperbolic.hh
  sot/core/flags.hh
  sot/core/sot-qr.hh
  sot/core/memory-task-sot.hh
  sot/core/sot-h.hh
  sot/core/sot.hh
  sot/core/rotation-simple.hh
  sot/core/weighted-sot.hh
  sot/core/solver-hierarchical-inequalities.hh
  sot/core/reader.hh
  sot/core/utils-windows.hh
  sot/core/time-stamp.hh
  sot/core/timer.hh
  sot/core/neck-limitation.hh
  sot/core/motion-period.hh
  sot/core/binary-int-to-uint.hh
  sot/core/clamp-workspace.hh
  sot/core/com-freezer.hh
  sot/core/gripper-control.hh
  sot/core/joint-limitator.hh
  sot/core/kalman.hh
  sot/core/mailbox-vector.hh
  sot/core/mailbox.hh
  sot/core/mailbox.hxx
  sot/core/periodic-call.hh
  sot/core/periodic-call-entity.hh
  )
INSTALL(FILES ${NEWHEADERS}
  DESTINATION include/sot/core
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
