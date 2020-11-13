from dynamic_graph import plug
from dynamic_graph.sot.core.feature_visual_point import FeatureVisualPoint
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.task import Task
from dynamic_graph.sot.core.visual_point_projecter import VisualPointProjecter


class MetaTaskVisualPoint(object):
    name = ''
    opPoint = ''
    dyn = 0
    task = 0
    feature = 0
    featureDes = 0
    proj = 0

    def opPointExist(self, opPoint):
        sigsP = filter(lambda x: x.getName().split(':')[-1] == opPoint,
                       self.dyn.signals())
        sigsJ = filter(lambda x: x.getName().split(':')[-1] == 'J' + opPoint,
                       self.dyn.signals())
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
        plug(
            self.dyn.signal(self.opPoint),
            self.opPointModif.signal('positionIN'))
        plug(
            self.dyn.signal('J' + self.opPoint),
            self.opPointModif.signal('jacobianIN'))
        self.opPointModif.activ = False

    def createFeatures(self):
        self.feature = FeatureVisualPoint('feature' + self.name)
        self.featureDes = FeatureVisualPoint('feature' + self.name + '_ref')
        self.feature.selec.value = '11'

    def createTask(self):
        self.task = Task('task' + self.name)

    def createGain(self):
        self.gain = GainAdaptive('gain' + self.name)
        self.gain.set(0.1, 0.1, 125e3)

    def createProj(self):
        self.proj = VisualPointProjecter('proj' + self.name)

    def plugEverything(self):
        self.feature.setReference(self.featureDes.name)
        plug(self.dyn.signal(self.opPoint), self.proj.signal('transfo'))
        plug(self.proj.signal('point2D'), self.feature.signal('xy'))
        plug(self.proj.signal('depth'), self.feature.signal('Z'))
        plug(self.dyn.signal('J' + self.opPoint), self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.task.error, self.gain.error)
        plug(self.gain.gain, self.task.controlGain)

    def __init__(self, name, dyn, opPoint, opPointRef='right-wrist'):
        self.name = name
        self.defineDynEntities(dyn)
        self.createOpPoint(opPoint, opPointRef)
        self.createOpPointModif()
        self.createFeatures()
        self.createTask()
        self.createGain()
        self.createProj()
        self.plugEverything()

    @property
    def ref(self):
        return self.featureDes.xy.value

    @ref.setter
    def ref(self, m):
        self.featureDes.xy.value = m

    @property
    def target(self):
        return self.proj.point3D

    @target.setter
    def target(self, m):
        self.proj.point3D.value = m

    @property
    def opmodif(self):
        if not self.opPointModif.activ:
            return False
        else:
            return self.opPointModif.getTransformation()

    @opmodif.setter
    def opmodif(self, m):
        if isinstance(m, bool) and not m:
            plug(self.dyn.signal(self.opPoint), self.proj.signal('transfo'))
            plug(
                self.dyn.signal('J' + self.opPoint), self.feature.signal('Jq'))
            self.opPointModif.activ = False
        else:
            if not self.opPointModif.activ:
                plug(
                    self.opPointModif.signal('position'),
                    self.proj.signal('transfo'))
                plug(
                    self.opPointModif.signal('jacobian'),
                    self.feature.signal('Jq'))
            self.opPointModif.setTransformation(m)
            self.opPointModif.activ = True

    def goto3D(self, point3D, gain=None, ref2D=None, selec=None):
        self.target = point3D
        if ref2D is not None:
            self.ref = ref2D
        if selec is not None:
            self.feature.selec.value = selec
        setGain(self.gain, gain)
