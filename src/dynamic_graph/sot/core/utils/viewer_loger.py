import glob
import os


class ViewerLoger:
    '''
    This class replace the robotviewer client and log the data sent to the
    viewer for future replay.

    Example of use:
    from viewer_loger import ViewerLoger
    robot.viewer = ViewerLoger(robot)

    '''
    def __init__(self, robot):
        self.robot = robot
        self.viewer = robot.viewer
        self.fileMap = {}
        for f in glob.glob('/tmp/view*.dat'):
            os.remove(f)

    def updateElementConfig(self, name, state):
        t = self.robot.state.time
        if name not in self.fileMap:
            self.fileMap[name] = open('/tmp/view_' + name + '.dat', 'w')
        self.fileMap[name].write("\t".join([str(f) for f in [
            t,
        ] + list(state)]) + '\n')
        self.viewer.updateElementConfig(name, state)
