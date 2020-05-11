#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

import unittest

import numpy as np
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier

gaze = tuple(((1.0, 0.0, 0.0, 0.025000000000000001), (0.0, 1.0, 0.0, 0.0),
              (0.0, 0.0, 1.0, 0.64800000000000002), (0.0, 0.0, 0.0, 1.0)))

Jgaze = tuple(
    ((1.0, 0.0, 0.0, 0.0, 0.64800000000000002,
      0.0), (0.0, 1.0, 0.0, -0.64800000000000002, 0.0,
             0.025000000000000001), (0.0, 0.0, 1.0, 0.0, -0.025000000000000001,
                                     0.0), (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
     (0.0, 0.0, 0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0, 1.0)))

I4 = ((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.))

I6 = ((1., 0., 0., 0., 0., 0.), (0., 1., 0., 0., 0., 0.),
      (0., 0., 1., 0., 0., 0.), (0., 0., 0., 1., 0., 0.),
      (0., 0., 0., 0., 1., 0.), (0., 0., 0., 0., 0., 1.))


class OpPointModifierTest(unittest.TestCase):
    def test_simple(self):
        op = OpPointModifier('op')
        op.setTransformation(I4)
        op.positionIN.value = I4
        op.jacobianIN.value = I6
        op.position.recompute(0)
        op.jacobian.recompute(0)

        self.assertEqual(op.getTransformation(), I4)
        self.assertEqual(op.position.value, I4)
        self.assertEqual(op.jacobian.value, I6)

    def test_translation(self):
        tx = 11.
        ty = 22.
        tz = 33.

        T = ((1., 0., 0., tx), (0., 1., 0., ty), (0., 0., 1., tz), (0., 0., 0.,
                                                                    1.))

        op = OpPointModifier('op2')
        op.setTransformation(T)
        op.positionIN.value = gaze
        op.jacobianIN.value = Jgaze
        op.position.recompute(1)
        op.jacobian.recompute(1)

        self.assertEqual(op.getTransformation(), T)

        # w_M_s = w_M_g * g_M_s
        w_M_g = np.asmatrix(gaze)
        g_M_s = np.asmatrix(T)
        w_M_s_ref = w_M_g * g_M_s
        w_M_s = np.asmatrix(op.position.value)

        # Check w_M_s == w_M_s_ref
        self.assertEqual(np.equal(w_M_s, w_M_s_ref).all(), True)

        twist = np.matrix([[1., 0., 0., 0., tz,
                            -ty], [0., 1., 0., -tz, 0., tx],
                           [0., 0., 1., ty, -tx, 0.], [0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 1., 0.], [0., 0., 0., 0., 0., 1.]])

        J = np.asmatrix(op.jacobian.value)
        J_ref = twist * Jgaze

        # Check w_M_s == w_M_s_ref
        self.assertEqual(np.equal(J, J_ref).all(), True)

    def test_rotation(self):
        T = ((0., 0., 1., 0.), (0., -1., 0., 0.), (1., 0., 0., 0.), (0., 0.,
                                                                     0., 1.))

        op = OpPointModifier('op3')
        op.setTransformation(T)
        op.positionIN.value = gaze
        op.jacobianIN.value = Jgaze
        op.position.recompute(1)
        op.jacobian.recompute(1)

        self.assertEqual(op.getTransformation(), T)

        # w_M_s = w_M_g * g_M_s
        w_M_g = np.asmatrix(gaze)
        g_M_s = np.asmatrix(T)
        w_M_s_ref = w_M_g * g_M_s
        w_M_s = np.asmatrix(op.position.value)

        # Check w_M_s == w_M_s_ref
        self.assertEqual(np.equal(w_M_s, w_M_s_ref).all(), True)

        twist = np.matrix([[0., 0., 1., 0., 0., 0.], [0., -1., 0., 0., 0., 0.],
                           [1., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 1.],
                           [0., 0., 0., 0., -1., 0.], [0., 0., 0., 1., 0.,
                                                       0.]])

        J = np.asmatrix(op.jacobian.value)
        J_ref = twist * Jgaze

        # Check w_M_s == w_M_s_ref
        self.assertEqual(np.equal(J, J_ref).all(), True)

    def test_rotation_translation(self):
        tx = 11.
        ty = 22.
        tz = 33.

        T = ((0., 0., 1., tx), (0., -1., 0., ty), (1., 0., 0., tz), (0., 0.,
                                                                     0., 1.))

        op = OpPointModifier('op4')
        op.setTransformation(T)
        op.positionIN.value = gaze
        op.jacobianIN.value = Jgaze
        op.position.recompute(1)
        op.jacobian.recompute(1)

        self.assertEqual(op.getTransformation(), T)

        # w_M_s = w_M_g * g_M_s
        w_M_g = np.asmatrix(gaze)
        g_M_s = np.asmatrix(T)
        w_M_s_ref = w_M_g * g_M_s
        w_M_s = np.asmatrix(op.position.value)

        # Check w_M_s == w_M_s_ref
        self.assertEqual(np.equal(w_M_s, w_M_s_ref).all(), True)

        twist = np.matrix(
            [[0., 0., 1., ty, -tx, 0.], [0., -1., 0., tz, 0., -tx],
             [1., 0., 0., 0., tz, -ty], [0., 0., 0., 0., 0., 1.],
             [0., 0., 0., 0., -1., 0.], [0., 0., 0., 1., 0., 0.]])

        J = np.asmatrix(op.jacobian.value)
        J_ref = twist * Jgaze

        # Check w_M_s == w_M_s_ref
        self.assertEqual(np.equal(J, J_ref).all(), True)


if __name__ == '__main__':
    unittest.main()
