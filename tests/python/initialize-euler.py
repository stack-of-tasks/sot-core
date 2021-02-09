#!/usr/bin/env python

import unittest

import dynamic_graph.sot.core.integrator_euler as ie


class OpPointModifierTest(unittest.TestCase):
    def test_initialize_ie(self):
        ent = ie.IntegratorEulerVectorDouble("ie")
        with self.assertRaises(RuntimeError) as cm:
            ent.initialize()
        self.assertEqual(str(cm.exception), 'The numerator or the denominator is empty.')


if __name__ == '__main__':
    unittest.main()
