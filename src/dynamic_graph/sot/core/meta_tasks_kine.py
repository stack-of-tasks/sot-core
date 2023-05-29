from dynamic_graph.sot.core.meta_tasks import MetaTaskCom
from dynamic_graph.sot.core.task import Task

class MetaTaskKineCom(MetaTaskCom):
    def __init__(self, dyn, name="com"):
        MetaTaskCom.__init__(self, dyn, name)
        self.task = Task("task" + name)
        self.plugTask()
