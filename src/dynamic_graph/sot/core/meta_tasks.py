from numpy import array, eye, matrix, ndarray

from dynamic_graph import plug
from dynamic_graph.sot.core import Flags
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.matrix_util import rpy2tr
from dynamic_graph.sot.core.meta_task_6d import toFlags  # noqa


class MetaTaskCom(object):
    def __init__(self, dyn, name="com"):
        self.dyn = dyn
        self.name = name
        # dyn.setProperty('ComputeCoM','true')

        self.feature = FeatureGeneric("feature" + name)
        self.featureDes = FeatureGeneric("featureDes" + name)
        self.gain = GainAdaptive("gain" + name)

        plug(dyn.com, self.feature.errorIN)
        plug(dyn.Jcom, self.feature.jacobianIN)
        self.feature.setReference(self.featureDes.name)

    def plugTask(self):
        self.task.add(self.feature.name)
        plug(self.task.error, self.gain.error)
        plug(self.gain.gain, self.task.controlGain)

    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self, v):
        self.featureDes.errorIN.value = v


# --- HELPER FUNCTIONS --------------------------------------------------------
def setGain(gain, val):
    if val is not None:
        if isinstance(val, int) or isinstance(val, float):
            gain.setConstant(val)
        elif len(val) == 1:
            gain.setConstant(val[0])
        elif len(val) == 3:
            gain.set(val[0], val[1], val[2])
        elif len(val) == 4:
            gain.setByPoint(val[0], val[1], val[2], val[3])


def generic6dReference(p):
    M = eye(4)
    if isinstance(p, (matrix, ndarray)) and p.size == 3:
        M[0:3, 3] = p
    elif isinstance(p, tuple) and len(p) == 3:
        M[0:3, 3] = p
    elif isinstance(p, (matrix, ndarray)) and p.shape == (4, 4):
        M = p
    elif isinstance(p, (matrix, tuple)) and len(p) == 4 == len(p[0]) == len(
        p[1]
    ) == len(p[2]) == len(p[3]):
        M = matrix(p)
    elif isinstance(p, (matrix, ndarray, tuple)) and len(p) == 6:
        M = array(rpy2tr(*p[3:7]))
        M[0:3, 3] = p[0:3]
    else:
        print("Position with other parameters ... todo")
    return M


def goto6d(task, position, gain=None, resetJacobian=True):
    M = generic6dReference(position)
    task.featureDes.position.value = array(M)
    task.feature.selec.value = Flags("111111")
    setGain(task.gain, gain)
    if (
        "resetJacobianDerivative" in task.task.__class__.__dict__.keys()
        and resetJacobian
    ):
        task.task.resetJacobianDerivative()


def gotoNd(task, position, selec=None, gain=None, resetJacobian=True):
    M = generic6dReference(position)
    if selec is not None:
        if not isinstance(selec, Flags):
            selec = Flags(selec)
        task.feature.selec.value = selec
    task.featureDes.position.value = array(M)
    setGain(task.gain, gain)
    if (
        "resetJacobianDerivative" in task.task.__class__.__dict__.keys()
        and resetJacobian
    ):
        task.task.resetJacobianDerivative()
