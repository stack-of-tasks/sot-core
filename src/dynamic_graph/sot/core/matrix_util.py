'''
Tiny matrix functions, taken from Oscar source code.
'''

from math import atan2
from random import random

from numpy import array, cos, inner, matrix, pi, sin, sqrt
from numpy.linalg import norm


# Convert matrix to tuple
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)


def vectorToTuple(M):
    if len(M.shape) == 1:
        return tuple(M.tolist())
    elif M.shape[0] == 1:
        return tuple(M.tolist()[0])
    else:
        return tuple(M.transpose().tolist()[0])


# Convert from Roll, Pitch, Yaw to transformation Matrix
def rpy2tr(r, p, y):
    mat = matrix(rotate('z', y)) * matrix(rotate('y', p)) * matrix(
        rotate('x', r))
    return matrixToTuple(mat)


# Get the distance btw the position components of 2 transf matrices
def distVector(M2, M1):
    px = M1[0][3] - M2[0][3]
    py = M1[1][3] - M2[1][3]
    pz = M1[2][3] - M2[2][3]
    return [px, py, pz]


# Obtain an orthonormal matrix SO3 using the given vector as its first column
# (it computes Gram Schmidt to obtain an orthonormal basis using the
#  first vector and 2 other 'random' vectors)


def generateOrthonormalM(v1):

    v2 = matrix([random(), random(), random()])
    v3 = matrix([random(), random(), random()])

    v1 = matrix(v1)
    e1 = v1 / norm(v1)

    u2 = v2 - inner(v2, e1) * e1
    e2 = u2 / norm(u2)

    u3 = v3 - inner(v3, e1) * e1 - inner(v3, e2) * e2
    e3 = u3 / norm(u3)

    e1 = e1.tolist()
    e2 = e2.tolist()
    e3 = e3.tolist()
    M = ((e1[0][0], e2[0][0], e3[0][0]), (e1[0][1], e2[0][1], e3[0][1]),
         (e1[0][2], e2[0][2], e3[0][2]))
    return M


# Convert from Transformation Matrix to Roll,Pitch,Yaw
def tr2rpy(M):
    m = sqrt(M[2][1]**2 + M[2][2]**2)
    p = atan2(-M[2][0], m)

    if abs(p - pi / 2) < 0.001:
        r = 0
        y = atan2(M[0][1], M[1][1])
    elif abs(p + pi / 2) < 0.001:
        r = 0
        y = -atan2(M[0][1], M[1][1])
    else:
        r = atan2(M[1][0], M[0][0])
        y = atan2(M[2][1], M[2][2])

    return [float(r), float(p), float(y)]


def matrixToRPY(M):
    '''
    Convert a 4x4 homogeneous matrix to a 6x1 rpy pose vector.
    '''
    rot = tr2rpy(M)
    return [M[0][3], M[1][3], M[2][3], rot[2], rot[1], rot[0]]


def RPYToMatrix(pr):
    '''
    Convert a 6x1 rpy pose vector to a 4x4 homogeneous matrix.
    '''
    M = array(rpy2tr(pr[3], pr[4], pr[5]))
    M[0:3, 3] = pr[0:3]
    return M


# Transformation Matrix corresponding to a rotation about x,y or z
def rotate(axis, ang):
    ''' eg. T=rot('x',pi/4): rotate pi/4 rad about x axis
    '''
    ca = cos(ang)
    sa = sin(ang)
    if axis == 'x':
        mat = ((1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1))
    elif axis == 'y':
        mat = ((ca, 0, sa, 0), (0, 1, 0, 0), (-sa, 0, ca, 0), (0, 0, 0, 1))
    elif axis == 'z':
        mat = ((ca, -sa, 0, 0), (sa, ca, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
    else:
        print('Axis should be: x,y or z only')
    return mat


def quaternionToMatrix(q):
    [qx, qy, qz, qw] = q
    R = [[
        1 - 2 * qy**2 - 2 * qz**2, 2 * qx * qy - 2 * qz * qw,
        2 * qx * qz + 2 * qy * qw
    ], [
        2 * qx * qy + 2 * qz * qw, 1 - 2 * qx**2 - 2 * qz**2,
        2 * qy * qz - 2 * qx * qw
    ], [
        2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw,
        1 - 2 * qx**2 - 2 * qy**2
    ]]
    return R
