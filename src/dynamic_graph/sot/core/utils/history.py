# This class is used to record the data during an execution and restore it at a
# given time isntant.

from dynamic_graph.sot.core.matrix_util import vectorToTuple
from numpy import matrix, pi


class History:
    def __init__(self, dynEnt, freq=100, zmpSig=None):
        self.q = list()
        self.qdot = list()
        self.zmp = list()
        self.freq = freq
        self.zmpSig = zmpSig
        self.dynEnt = dynEnt
        self.withZmp = (self.zmpSig is not None) and ("waist" in map(lambda x: x.name, self.dynEnt.signals()))

    def record(self):
        i = self.dynEnt.position.time
        if i % self.freq == 0:
            self.q.append(self.dynEnt.position.value)
            self.qdot.append(self.dynEnt.velocity.value)
            if self.withZmp:
                waMwo = matrix(self.dynEnt.waist.value).I
                wo_z = matrix(self.zmpSig.value + (1, )).T
                self.zmp.append(list(vectorToTuple(waMwo * wo_z)))

    def restore(self, t):
        t = int(t / self.freq)
        print("robot.set(", self.q[t], ")")
        print("robot.setVelocity(", self.qdot[t], ")")
        print("T0 = ", t)
        print("robot.state.time = T0")
        print("[ t.feature.position.recompute(T0) for t in refreshTaskList]")
        print("attime.fastForward(T0)")

    def dumpToOpenHRP(self, baseName="dyninv", sample=1):
        filePos = open(baseName + '.pos', 'w')
        fileRPY = open(baseName + '.hip', 'w')
        fileWaist = open(baseName + '.waist', 'w')
        sampleT = 0.005
        for nT, q in enumerate(self.q):
            fileRPY.write(str(sampleT * nT) + ' ' + str(q[3]) + ' ' + str(q[4]) + ' ' + str(q[5]) + '\n')
            fileWaist.write(
                str(sampleT * nT) + ' ' + str(q[0]) + ' ' + str(q[1]) + ' ' + str(q[2]) + ' ' + str(q[3]) + ' ' +
                str(q[4]) + ' ' + str(q[5]) + '\n')
            filePos.write(str(sampleT * nT) + ' ')
            for j in range(6, 36):
                filePos.write(str(q[j]) + ' ')
            filePos.write(10 * ' 0' + '\n')
        if self.withZmp:
            fileZMP = open(baseName + '.zmp', 'w')
            for nT, z in enumerate(self.zmp):
                fileZMP.write(str(sampleT * nT) + ' ' + str(z[0]) + ' ' + str(z[1]) + ' ' + str(z[2]) + '\n')

        filePos0 = open(baseName + '_pos0.py', 'w')
        filePos0.write("dyninv_posinit = '")
        q0 = self.q[0]
        for x in q0[6:36]:
            filePos0.write(str(x * 180.0 / pi) + ' ')
        filePos0.write("   0 0 0 0 0 0 0 0 0 0  '")
