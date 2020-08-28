class RobotViewerFaked:
    def update(*args):
        None

    def updateElementConfig(*args):
        None


def stateFullSize(robot, additionalData=()):
    return [float(val) for val in robot.state.value + additionalData]


def refreshView(robot):
    if robot.name == 'robot':
        name = 'hrp'
    if robot.name == 'robot_device':
        name = 'hrp'
    else:
        name = robot.name
    robot.viewer.updateElementConfig(name, robot.stateFullSize())

    for f in robot.displayList:
        f()


def incrementView(robot, dt):
    '''Increment then refresh.'''
    robot.incrementNoView(dt)
    robot.refresh()


def setView(robot, *args):
    '''Set robot config then refresh.'''
    robot.setNoView(*args)
    # print('view')
    robot.refresh()


def addRobotViewer(robot, **args):
    '''
    Arguments are:
      * small=False
      * server='XML-RPC' == { 'XML-RPC' | 'CORBA' }
      * verbose=True
    '''
    verbose = args.get('verbose', True)
    small = args.get('small', False)
    small_extra = args.get('small_extra', 10)
    server = args.get('server', 'XML-RPC')

    try:
        import robotviewer
        RobotClass = robot.__class__

        if small:
            if verbose:
                print('using a small robot')
            RobotClass.stateFullSize = lambda x: \
                stateFullSize(x, small_extra * (0.0, ))
        else:
            RobotClass.stateFullSize = stateFullSize

        robot.viewer = robotviewer.client(server)

        RobotClass.refresh = refreshView
        RobotClass.incrementNoView = RobotClass.increment
        RobotClass.increment = incrementView
        RobotClass.setNoView = RobotClass.set
        RobotClass.set = setView

        robot.displayList = []

        # Check the connection
        if args.get('dorefresh', True):
            robot.refresh()

    except Exception:
        if verbose:
            print("No robot viewer, sorry.")
        robot.viewer = RobotViewerFaked()


# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# Add a visual output when an event is called.
class VisualPinger:
    def __init__(self, viewer):
        self.pos = 1
        self.viewer = viewer
        self.refresh()

    def refresh(self):
        self.viewer.updateElementConfig('pong',
                                        [0, 0.9, self.pos * 0.1, 0, 0, 0])

    def __call__(self):
        self.pos += 1
        self.refresh()


def updateComDisplay(robot, comsig, objname='com'):
    if comsig.time > 0:
        robot.viewer.updateElementConfig(
            objname, [comsig.value[0], comsig.value[1], 0, 0, 0, 0])
