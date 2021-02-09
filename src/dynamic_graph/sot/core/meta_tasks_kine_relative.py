from dynamic_graph import plug
from dynamic_graph.sot.core import Flags
from dynamic_graph.sot.core.feature_point6d_relative import FeaturePoint6dRelative
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d, toFlags  # noqa kept for backward compatibility
from dynamic_graph.sot.core.meta_tasks import generic6dReference, setGain
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier


class MetaTaskKine6dRel(MetaTask6d):

    opPointBase = ''

    def createOpPointBase(self, opPointBase, opPointRefBase='left-wrist'):
        self.opPointBase = opPointBase
        if self.opPointExist(opPointBase):
            return
        self.dyn.createOpPoint(opPointBase, opPointRefBase)

    def createOpPointModifBase(self):
        self.opPointModifBase = OpPointModifier('opmodifBase' + self.name)
        plug(self.dyn.signal(self.opPointBase), self.opPointModifBase.signal('positionIN'))
        plug(self.dyn.signal('J' + self.opPointBase), self.opPointModifBase.signal('jacobianIN'))
        self.opPointModifBase.activ = False

    def createFeatures(self):
        self.feature = FeaturePoint6dRelative('featureRel' + self.name)
        self.featureDes = FeaturePoint6dRelative('featureRel' + self.name + '_ref')
        self.feature.selec.value = Flags('111111')
        self.feature.frame('current')

    def plugEverything(self):
        self.feature.setReference(self.featureDes.name)
        plug(self.dyn.signal(self.opPoint), self.feature.signal('position'))
        plug(self.dyn.signal('J' + self.opPoint), self.feature.signal('Jq'))
        plug(self.dyn.signal(self.opPointBase), self.feature.signal('positionRef'))
        plug(self.dyn.signal('J' + self.opPointBase), self.feature.signal('JqRef'))
        self.task.add(self.feature.name)
        plug(self.task.error, self.gain.error)
        plug(self.gain.gain, self.task.controlGain)

    def keep(self):
        self.feature.position.recompute(self.dyn.position.time)
        self.feature.keep()

    def __init__(self, name, dyn, opPoint, opPointBase, opPointRef='right-wrist', opPointRefBase='left-wrist'):
        self.name = name
        self.defineDynEntities(dyn)
        self.createOpPoint(opPoint, opPointRef)
        self.createOpPointBase(opPointBase, opPointRefBase)
        self.createOpPointModif()
        self.createOpPointModifBase()
        self.createFeatures()
        self.createTask()
        self.createGain()
        self.plugEverything()

    @property
    def opmodifBase(self):
        if not self.opPointModifBase.activ:
            return False
        else:
            return self.opPointModifBase.getTransformation()

    @opmodifBase.setter
    def opmodifBase(self, m):
        if isinstance(m, bool) and not m:
            plug(self.dyn.signal(self.opPointBase), self.feature.signal('positionRef'))
            plug(self.dyn.signal('J' + self.opPointBase), self.feature.signal('JqRef'))
            self.opPointModifBase.activ = False
        else:
            if not self.opPointModifBase.activ:
                plug(self.opPointModifBase.signal('position'), self.feature.positionRef)
                plug(self.opPointModifBase.signal('jacobian'), self.feature.JqRef)
            self.opPointModifBase.setTransformation(m)
            self.opPointModifBase.activ = True


# --- HELPER FUNCTIONS --------------------------------------------------------


def goto6dRel(task, position, positionRef, gain=None, resetJacobian=True):
    M = generic6dReference(position)
    MRef = generic6dReference(positionRef)
    task.featureDes.position.value = matrixToTuple(M)
    task.featureDes.positionRef.value = matrixToTuple(MRef)
    task.feature.selec.value = Flags("111111")
    setGain(task.gain, gain)
    if 'resetJacobianDerivative' in task.task.__class__.__dict__.keys() and resetJacobian:
        task.task.resetJacobianDerivative()


def gotoNdRel(task, position, positionRef, selec=None, gain=None, resetJacobian=True):
    M = generic6dReference(position)
    MRef = generic6dReference(positionRef)
    if selec is not None:
        if not isinstance(selec, Flags):
            selec = Flags(selec)
        task.feature.selec.value = selec
    task.featureDes.position.value = matrixToTuple(M)
    task.featureDes.positionRef.value = matrixToTuple(MRef)
    setGain(task.gain, gain)
    if 'resetJacobianDerivative' in task.task.__class__.__dict__.keys() and resetJacobian:
        task.task.resetJacobianDerivative()


"""

Documentation

Inherited from MetaTask6d.

The aim of this MetaTask is to give a simple and immediate interface to
implement a relative task between two
operational points of the robot. The new variable "opPointBase" represents
in fact the second operational point (the
first is inherited from the father class).

It's been decided to reuse (so not to redefine) all methodes from MetaTask6d
related to the opPoint to implement the
behaviour of one of the two points (called "Other" from now on) and to
reimplement in a intuitive way the same
functions for the second point ("Ref").


Utilization

It should be noticed that both feature and reference are defined as a
couple of signals, while normally it would be
enough define the reference as one signal that represents the diplacement
between the two positions. Nevertheless this
redundant approach allows to a very intuitive and safe usage of the class
because the references can be set just using
the current position of the two operational points.
For this reason all the goTo functions have been redefined.

"""
