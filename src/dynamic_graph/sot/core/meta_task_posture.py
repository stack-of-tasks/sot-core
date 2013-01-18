from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate
from numpy import matrix, identity, zeros, eye


class MetaTaskPosture(object):
    postureRange = { \
        "rleg": range(6,12), \
        "lleg": range(12,18), \
        "chest": range(18,20), \
        "head": range(20,22), \
        "rarm": range(22,28), \
        "rhand": [28], \
        "larm": range(29,35), \
        "lhand": [35], \
            }
    nbDof = None

    def __init__(self,dyn,name="posture"):
        self.dyn=dyn
        self.name=name
        self.feature    = FeatureGeneric('feature'+name)
        self.featureDes = FeatureGeneric('featureDes'+name)
        self.gain = GainAdaptive('gain'+name)
        plug(dyn.position,self.feature.errorIN)
        robotDim = len(dyn.position.value)
        self.feature.jacobianIN.value = matrixToTuple( identity(robotDim) )
        self.feature.setReference(self.featureDes.name)

    def plugTask(self):
        self.task.add(self.feature.name)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)

    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self,v):
        self.featureDes.errorIN.value = v

    def gotoq(self,gain=None,qdes=None,**kwargs):
        act=list()
        if qdes!=None:
            if isinstance(qdes,tuple): qdes=matrix(qdes).T
        else:
            if MetaTaskPosture.nbDof==None:
                MetaTaskPosture.nbDof = len(self.feature.errorIN.value)
            qdes = zeros((MetaTaskPosture.nbDof,1))

        for limbName,jointValues in kwargs.items():
            limbRange = self.postureRange[limbName]
            act += limbRange
            if jointValues!=[]:
                if isinstance(jointValues,matrix):
                    qdes[limbRange,0] = vectorToTuple(jointValues)
                else: qdes[limbRange,0] = jointValues
        self.ref = vectorToTuple(qdes)
        self.feature.selec.value = toFlags(act)
        setGain(self.gain,gain)

class MetaTaskKinePosture(MetaTaskPosture):
    def __init__(self,dyn,name="posture"):
        MetaTaskPosture.__init__(self,dyn,name)
        self.task = Task('task'+name)
        self.plugTask()
