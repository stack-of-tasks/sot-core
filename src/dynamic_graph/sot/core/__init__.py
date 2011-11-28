from robot_simu import RobotSimu
from feature_point6d import FeaturePoint6d
from feature_position import FeaturePosition
from feature_posture import FeaturePosture
from feature_point6d_relative import FeaturePoint6dRelative
from feature_generic import FeatureGeneric
from feature_joint_limits import FeatureJointLimits
from feature_visual_point import FeatureVisualPoint
from task import Task
from task_pd import TaskPD
from constraint import Constraint
from gain_adaptive import GainAdaptive
from joint_limitator import JointLimitator
from sot import SOT
#from smooth_reach import SmoothReach

# --- PATCH --------------------------------------------------------------------
'''
This patch is used to keep the behavior of the sdes signal, while moving to the
new formulation using the getter getReference and the setter setReference. This
patch is temporary, and will be maintained during the time needed to convert
all the depending application to the new syntaxt.
'''

class SdesPatch(object):
    def __init__(self,feature):
        self.feature = feature
    @property
    def value(self):
        print 'Warning, deprecated: signal sdes for features is deprecated. Use the g/setReference functions instead.'
        return self.feature.getReference()
    @value.setter
    def value(self,v):
        print 'Warning, deprecated: signal sdes for features is deprecated. Use the g/setReference functions instead.'
        self.feature.setReference(v)

def patchFeature(FeatureClass):
    FeatureClass.initWithoutSdesPatch = FeatureClass.__init__ 
    def FeatureInitPatched(self,name):
        FeatureClass.initWithoutSdesPatch(self,name)
        self.sdes = SdesPatch(self)
    FeatureClass.__init__ = FeatureInitPatched

patchFeature(FeaturePoint6d)
patchFeature(FeaturePosture)
patchFeature(FeaturePoint6dRelative)
patchFeature(FeatureGeneric)
patchFeature(FeatureJointLimits)
patchFeature(FeatureVisualPoint)
# --- /PATCH -------------------------------------------------------------------

RobotSimu('')
FeaturePoint6d('')
FeaturePosture('')
FeaturePoint6dRelative('')
FeatureGeneric('')
FeatureJointLimits('')
FeatureVisualPoint('')
Task('')
TaskPD('')
Constraint('')
GainAdaptive('')
JointLimitator('')
SOT('')

from op_point_modifier import OpPointModifier
OpPointModifier('')

from math_small_entities import *





