#!/usr/bin/env python

import unittest

import numpy as np
from dynamic_graph.sot.core import matrix_util as mod


class MatrixUtilTest(unittest.TestCase):
    def test_matrix_to_tuple(self):
        mat = ((1, 2), (3, 4))
        self.assertEqual(mat, mod.matrixToTuple(np.matrix(mat)))
        self.assertEqual(mat, mod.matrixToTuple(np.array(mat)))

    def test_vector_to_tuple(self):
        vec = (1, 2, 3, 4)
        self.assertEqual(vec, mod.vectorToTuple(np.matrix(vec)))
        self.assertEqual(vec, mod.vectorToTuple(np.array(vec)))

    def test_rpy_tr(self):
        for rpy, tr in [
            ((0, 0, 0), mod.matrixToTuple(np.identity(4))),
            ((np.pi, 0, 0), ((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1))),
            ((0, np.pi, 0), ((-1, 0, 0, 0), (0, 1, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1))),
            ((0, 0, np.pi), ((-1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))),
        ]:
            np.testing.assert_allclose(tr, mod.rpy2tr(*rpy), atol=1e-15)
            # np.testing.assert_allclose(rpy, mod.tr2rpy(tr), atol=1e-15)
            # np.testing.assert_allclose(tr, mod.rpy2tr(*mod.tr2rpy(tr)), atol=1e-15)
            # np.testing.assert_allclose(rpy, mod.tr2rpy(mod.rpy2tr(*rpy)), atol=1e-15)

    def test_matrix_rpy(self):
        for mat, rpy in [
            (mod.matrixToTuple(np.identity(4)), (0, 0, 0, 0, 0, 0)),
            (mod.matrixToTuple(-np.identity(4)), (0, 0, 0, -np.pi, 0, -np.pi)),
        ]:
            np.testing.assert_allclose(rpy, mod.matrixToRPY(mat))
            # np.testing.assert_allclose(mat, mod.RPYToMatrix(rpy))
            np.testing.assert_allclose(rpy, mod.matrixToRPY(mod.RPYToMatrix(rpy)))
            # np.testing.assert_allclose(mat, mod.RPYToMatrix(mod.matrixToRPY(mat)))


if __name__ == '__main__':
    unittest.main()
