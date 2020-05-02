#!/usr/bin/env python

import unittest

import dynamic_graph as dgpy
import dynamic_graph.sot.core.integrator_euler as ie


class OpPointModifierTest(unittest.TestCase):
    def test_initialize_ie(self):
        ent = ie.IntegratorEulerVectorDouble("ie")
        with self.assertRaises(dgpy.dgpyError) as cm:
            ent.initialize()
        self.assertEqual(
            str(cm.exception),
            'In SignalPtr: SIN ptr not set. (in signal <sotIntegratorAbstract(ie)::input(vector)::sin>)'
        )


if __name__ == '__main__':
    unittest.main()
