# 
#  Copyright 
# 

SET(${PROJECT_NAME}_HEADERS
sot-core-api.h

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
signal-cast.h

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
matrix-homogeneous.h
matrix-rotation.h
matrix-twist.h
op-point-modifier.h
vector-quaternion.h
vector-roll-pitch-yaw.h
vector-rotation.h
vector-utheta.h

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
periodic-call.h
utils-windows.h
time-stamp.h
timer.h
periodic-call-entity.h
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
