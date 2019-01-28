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

SET(NEWHEADERS 
  sot/core/api.hh
  sot/core/abstract-sot-external-interface.hh
  sot/core/device.hh
  sot/core/robot-simu.hh
  sot/core/matrix-geometry.hh
  sot/core/matrix-svd.hh
  sot/core/contiifstream.hh
  sot/core/debug.hh
  sot/core/event.hh
  sot/core/exception-abstract.hh
  sot/core/exception-dynamic.hh
  sot/core/exception-factory.hh
  sot/core/exception-feature.hh
  sot/core/exception-signal.hh
  sot/core/exception-task.hh
  sot/core/exception-tools.hh
  sot/core/binary-op.hh
  sot/core/derivator.hh
  sot/core/latch.hh
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
  sot/core/op-point-modifier.hh
  sot/core/feature-point6d.hh
  sot/core/feature-vector3.hh
  sot/core/feature-abstract.hh
  sot/core/feature-generic.hh
  sot/core/feature-joint-limits.hh
  sot/core/feature-1d.hh
  sot/core/feature-point6d-relative.hh
  sot/core/feature-visual-point.hh
  sot/core/visual-point-projecter.hh
  sot/core/feature-posture.hh
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
  sot/core/memory-task-sot.hh
  sot/core/sot.hh
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
  sot/core/trajectory.hh
  sot/core/switch.hh
  sot/core/variadic-op.hh
  sot/core/robot-utils.hh
  )
INSTALL(FILES ${NEWHEADERS}
  DESTINATION include/sot/core
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
