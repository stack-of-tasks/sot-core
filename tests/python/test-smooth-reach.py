#!/usr/bin/env python

import unittest
from dynamic_graph.sot.core.smooth_reach import SmoothReach


class SmoothReachTest(unittest.TestCase):
    def test_smooth_reach(self):
        sr = SmoothReach("banana")
        self.assertIn("input(vector)::start (Type Cst)", str(sr.start))
        self.assertIn("output(vector)::goal (Type Fun)", str(sr.goal))


if __name__ == '__main__':
    unittest.main()
