from dynamic_graph import plug
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive

# TODO: this function is imported from meta_tasks_kine in several places,
# whereas it is defined in meta_tasks
from dynamic_graph.sot.core.meta_tasks import gotoNd  # noqa
from dynamic_graph.sot.core.meta_tasks import MetaTaskCom
from dynamic_graph.sot.core.task import Task

class MetaTaskKineCom(MetaTaskCom):
    def __init__(self, dyn, name="com"):
        MetaTaskCom.__init__(self, dyn, name)
        self.task = Task("task" + name)
        self.plugTask()
