#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

import unittest

import numpy as np
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier

gaze = np.array((((1.0, 0.0, 0.0, 0.025), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.648), (0.0, 0.0, 0.0, 1.0))))

Jgaze = np.array(
    (((1.0, 0.0, 0.0, 0.0, 0.648, 0.0), (0.0, 1.0, 0.0, -0.648, 0.0, 0.025), (0.0, 0.0, 1.0, 0.0, -0.025, 0.0),
      (0.0, 0.0, 0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0, 1.0))))

I4 = np.array(((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.)))

I6 = np.array(((1., 0., 0., 0., 0., 0.), (0., 1., 0., 0., 0., 0.), (0., 0., 1., 0., 0., 0.), (0., 0., 0., 1., 0., 0.),
               (0., 0., 0., 0., 1., 0.), (0., 0., 0., 0., 0., 1.)))


class OpPointModifierTest(unittest.TestCase):
    def test_simple(self):
        op = OpPointModifier('op')
        op.setTransformation(I4)
        op.positionIN.value = I4
        op.jacobianIN.value = I6
        op.position.recompute(0)
        op.jacobian.recompute(0)

        self.assertTrue((op.getTransformation() == I4).all())
        self.assertTrue((op.position.value == I4).all())
        self.assertTrue((op.jacobian.value == I6).all())

    def test_translation(self):
        tx = 11.
        ty = 22.
        tz = 33.

        T = np.array(((1., 0., 0., tx), (0., 1., 0., ty), (0., 0., 1., tz), (0., 0., 0., 1.)))

        op = OpPointModifier('op2')
        op.setTransformation(T)
        op.positionIN.value = gaze
        op.jacobianIN.value = Jgaze
        op.position.recompute(1)
        op.jacobian.recompute(1)

        self.assertTrue((op.getTransformation() == T).all())

        # w_M_s = w_M_g * g_M_s
        w_M_g = gaze
        g_M_s = T
        w_M_s_ref = w_M_g.dot(g_M_s)
        w_M_s = op.position.value

        # Check w_M_s == w_M_s_ref
        self.assertTrue((w_M_s == w_M_s_ref).all())

        twist = np.array([[1., 0., 0., 0., tz, -ty], [0., 1., 0., -tz, 0., tx], [0., 0., 1., ty, -tx, 0.],
                          [0., 0., 0., 1., 0., 0.], [0., 0., 0., 0., 1., 0.], [0., 0., 0., 0., 0., 1.]])

        J = op.jacobian.value
        J_ref = twist.dot(Jgaze)

        # Check w_M_s == w_M_s_ref
        self.assertTrue((J == J_ref).all())

    def test_rotation(self):
        T = np.array(((0., 0., 1., 0.), (0., -1., 0., 0.), (1., 0., 0., 0.), (0., 0., 0., 1.)))

        op = OpPointModifier('op3')
        op.setTransformation(T)
        op.positionIN.value = gaze
        op.jacobianIN.value = Jgaze
        op.position.recompute(1)
        op.jacobian.recompute(1)

        self.assertTrue((op.getTransformation() == T).all())

        # w_M_s = w_M_g * g_M_s
        w_M_g = gaze
        g_M_s = T
        w_M_s_ref = w_M_g.dot(g_M_s)
        w_M_s = op.position.value

        # Check w_M_s == w_M_s_ref
        self.assertTrue((w_M_s == w_M_s_ref).all())

        twist = np.array([[0., 0., 1., 0., 0., 0.], [0., -1., 0., 0., 0., 0.], [1., 0., 0., 0., 0., 0.],
                          [0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., -1., 0.], [0., 0., 0., 1., 0., 0.]])

        J = op.jacobian.value
        J_ref = twist.dot(Jgaze)

        # Check w_M_s == w_M_s_ref
        self.assertTrue((J == J_ref).all())

    def test_rotation_translation(self):
        tx = 11.
        ty = 22.
        tz = 33.

        T = np.array(((0., 0., 1., tx), (0., -1., 0., ty), (1., 0., 0., tz), (0., 0., 0., 1.)))

        op = OpPointModifier('op4')
        op.setTransformation(T)
        op.positionIN.value = gaze
        op.jacobianIN.value = Jgaze
        op.position.recompute(1)
        op.jacobian.recompute(1)

        self.assertTrue((op.getTransformation() == T).all())

        # w_M_s = w_M_g * g_M_s
        w_M_g = gaze
        g_M_s = T
        w_M_s_ref = w_M_g.dot(g_M_s)
        w_M_s = op.position.value

        # Check w_M_s == w_M_s_ref
        self.assertTrue((w_M_s == w_M_s_ref).all())

        twist = np.array([[0., 0., 1., ty, -tx, 0.], [0., -1., 0., tz, 0., -tx], [1., 0., 0., 0., tz, -ty],
                          [0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., -1., 0.], [0., 0., 0., 1., 0., 0.]])

        J = op.jacobian.value
        J_ref = twist.dot(Jgaze)

        # Check w_M_s == w_M_s_ref
        self.assertTrue((J == J_ref).all())


if __name__ == '__main__':
    unittest.main()
