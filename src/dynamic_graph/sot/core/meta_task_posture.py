from dynamic_graph import plug
from dynamic_graph.sot.core import Flags
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple
from dynamic_graph.sot.core.meta_task_6d import toFlags  # noqa
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.task import Task
from numpy import identity, matrix, zeros


class MetaTaskPosture(object):
    postureRange = {
        "rleg": range(6, 12),
        "rhip": range(6, 9),
        "rknee": [9],
        "rankle": range(10, 12),
        "lleg": range(12, 18),
        "lhip": range(12, 15),
        "lknee": [15],
        "lankle": range(16, 18),
        "chest": range(18, 20),
        "head": range(20, 22),
        "rarm": range(22, 28),
        "rshoulder": range(22, 25),
        "relbow": [25],
        "rwrist": range(26, 28),
        "rhand": [28],
        "larm": range(29, 35),
        "lshoulder": range(29, 32),
        "lelbow": [32],
        "lwrist": range(33, 35),
        "lhand": [35],
    }
    nbDof = None

    def __init__(self, dyn, name="posture"):
        self.dyn = dyn
        self.name = name
        self.feature = FeatureGeneric("feature" + name)
        self.featureDes = FeatureGeneric("featureDes" + name)
        self.gain = GainAdaptive("gain" + name)
        plug(dyn.position, self.feature.errorIN)
        robotDim = len(dyn.position.value)
        self.feature.jacobianIN.value = matrixToTuple(identity(robotDim))
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

    def gotoq(self, gain=None, qdes=None, **kwargs):
        act = list()
        if qdes is not None:
            if isinstance(qdes, tuple):
                qdes = matrix(qdes).T
        else:
            if MetaTaskPosture.nbDof is None:
                MetaTaskPosture.nbDof = len(self.feature.errorIN.value)
            qdes = zeros((MetaTaskPosture.nbDof, 1))

        act = [
            False,
        ] * MetaTaskPosture.nbDof
        for limbName, jointValues in kwargs.items():
            limbRange = self.postureRange[limbName]
            for i in limbRange:
                act[i] = True
            if jointValues != []:
                if isinstance(jointValues, matrix):
                    qdes[limbRange, 0] = vectorToTuple(jointValues)
                else:
                    qdes[limbRange, 0] = jointValues
        self.ref = vectorToTuple(qdes)
        if len(act) > 0:
            self.feature.selec.value = Flags(act)
        setGain(self.gain, gain)


class MetaTaskKinePosture(MetaTaskPosture):
    def __init__(self, dyn, name="posture"):
        MetaTaskPosture.__init__(self, dyn, name)
        self.task = Task("task" + name)
        self.plugTask()
