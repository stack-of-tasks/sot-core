from dynamic_graph import plug
from dynamic_graph.sot.core import Flags
from dynamic_graph.sot.core.feature_point6d import FeaturePoint6d
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.task import Task


def toFlags(arr):
    from warnings import warn
    warn("This function is deprecated. Please, use Flags directly.")
    return Flags(arr)

class MetaTask6d(object):
    name = ''
    opPoint = ''
    dyn = 0
    task = 0
    feature = 0
    featureDes = 0

    def opPointExist(self, opPoint):
        sigsP = [x for x in self.dyn.signals() if x.getName().split(':')[-1] == opPoint]
        sigsJ = [x for x in self.dyn.signals() if x.getName().split(':')[-1] == 'J' + opPoint]
        return len(sigsP) == 1 & len(sigsJ) == 1

    def defineDynEntities(self, dyn):
        self.dyn = dyn

    def createOpPoint(self, opPoint, opPointRef='right-wrist'):
        self.opPoint = opPoint
        if self.opPointExist(opPoint):
            return
        self.dyn.createOpPoint(opPoint, opPointRef)

    def createOpPointModif(self):
        self.opPointModif = OpPointModifier('opmodif' + self.name)
        plug(self.dyn.signal(self.opPoint), self.opPointModif.signal('positionIN'))
        plug(self.dyn.signal('J' + self.opPoint), self.opPointModif.signal('jacobianIN'))
        self.opPointModif.activ = False

    def createFeatures(self):
        self.feature = FeaturePoint6d('feature' + self.name)
        self.featureDes = FeaturePoint6d('feature' + self.name + '_ref')
        self.feature.selec.value = Flags('111111')
        self.feature.frame('current')

    def createTask(self):
        self.task = Task('task' + self.name)

    def createGain(self):
        self.gain = GainAdaptive('gain' + self.name)
        self.gain.set(0.1, 0.1, 125e3)

    def plugEverything(self):
        self.feature.setReference(self.featureDes.name)
        plug(self.dyn.signal(self.opPoint), self.feature.signal('position'))
        plug(self.dyn.signal('J' + self.opPoint), self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.task.error, self.gain.error)
        plug(self.gain.gain, self.task.controlGain)

    def keep(self):
        self.feature.position.recompute(self.dyn.position.time)
        self.feature.keep()

    def __init__(self, name, dyn, opPoint, opPointRef='right-wrist'):
        self.name = name
        self.defineDynEntities(dyn)
        self.createOpPoint(opPoint, opPointRef)
        self.createOpPointModif()
        self.createFeatures()
        self.createTask()
        self.createGain()
        self.plugEverything()

    @property
    def ref(self):
        return self.featureDes.position.value

    @ref.setter
    def ref(self, m):
        self.featureDes.position.value = m

    @property
    def opmodif(self):
        if not self.opPointModif.activ:
            return False
        else:
            return self.opPointModif.getTransformation()

    @opmodif.setter
    def opmodif(self, m):
        if isinstance(m, bool) and not m:
            plug(self.dyn.signal(self.opPoint), self.feature.signal('position'))
            plug(self.dyn.signal('J' + self.opPoint), self.feature.signal('Jq'))
            self.opPointModif.activ = False
        else:
            if not self.opPointModif.activ:
                plug(self.opPointModif.signal('position'), self.feature.position)
                plug(self.opPointModif.signal('jacobian'), self.feature.Jq)
            self.opPointModif.setTransformation(m)
            self.opPointModif.activ = True
