# This class embeds a given loop function (generally by using a decorator) into
# a dedicated thread, whose execution can be stoped or run on demand from the
# script thread.
#
# To use the previous class, a 'loop' function has to be defined.
# Everything will be embedded by using the decorator below. Just
# use it as:
#   >>> @loopInThread
#   >>> def Runner():
#   >>>    to what you want here
#   >>> runner = Runner()
#   >>> runner.pause()/play()/quit() ...

import threading
import time

from dynamic_graph.script_shortcuts import optionalparentheses


class ThreadInterruptibleLoop(threading.Thread):
    isQuit = False
    isPlay = False
    sleepTime = 1e-3
    previousHandler = None
    isOnce = 0
    isRunning = False
    iter = 0

    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True

    def quit(self):
        self.isQuit = True

    def setPlay(self, mode):
        self.isPlay = mode

    def play(self):
        if not self.isRunning:
            self.start()
        self.isOnce = False
        self.setPlay(True)

    def pause(self):
        self.setPlay(False)

    def once(self):
        self.isOnce = True
        self.setPlay(True)

    def run(self):
        self.isQuit = False
        self.isRunning = True
        while not self.isQuit:
            if self.isPlay:
                self.loop()
                self.iter += 1
                if self.isOnce:
                    self.pause()
            time.sleep(self.sleepTime)
        self.isRunning = False
        print("Thread loop will now end.")

    def start(self):
        self.setPlay(True)
        threading.Thread.start(self)

    def restart(self):
        self.join()
        self.play()
        self.setSigHandler()
        threading.Thread.start(self)

    def loop(self):
        None


# To use the previous class, a 'loop' function has to be define.
# Everything will be embedded by using the decorator below. Just
# use it as:
#   >>> @loopInThread
#   >>> def Runner():
#   >>>    to what you want here
#   >>> runner = Runner()
#   >>> runner.pause()/play()/quit() ...
def loopInThread(funLoop):
    class ThreadViewer(ThreadInterruptibleLoop):
        def __init__(self):
            ThreadInterruptibleLoop.__init__(self)

        #            self.start()

        def loop(self):
            funLoop()

    return ThreadViewer


# Define the 4 classical shortcuts to control the loop.
def loopShortcuts(runner):
    @optionalparentheses
    def go():
        runner.play()

    @optionalparentheses
    def stop():
        runner.pause()

    @optionalparentheses
    def next():
        runner.loop()

    class NextInc:
        def __add__(self, x):
            for i in range(x):
                next()

    n = NextInc()
    return [go, stop, next, n]
