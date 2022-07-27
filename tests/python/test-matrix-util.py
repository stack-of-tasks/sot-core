#!/usr/bin/env python

import unittest

import numpy as np
import dynamic_graph.sot.core.matrix_util as mod


class MatrixUtilTest(unittest.TestCase):
    def test_matrix_to_tuple(self):
        mat = ((1, 2), (3, 4))
        self.assertEqual(mat, mod.matrixToTuple(np.matrix(mat)))

    def test_array_to_tuple(self):
        mat = ((1, 2), (3, 4))
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

    def test_matrix_rpy(self):
        for mat, rpy in [
            (mod.matrixToTuple(np.identity(4)), (0, 0, 0, 0, 0, 0)),
            (mod.matrixToTuple(-np.identity(4)), (0, 0, 0, -np.pi, 0, -np.pi)),
        ]:
            np.testing.assert_allclose(rpy, mod.matrixToRPY(mat))
            # np.testing.assert_allclose(mat, mod.RPYToMatrix(rpy))
            np.testing.assert_allclose(rpy, mod.matrixToRPY(mod.RPYToMatrix(rpy)))

        def test_rotate(self):
            for axis, angle, mat in [
                (
                    "x",
                    np.pi,
                    ((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1)),
                ),
                (
                    "y",
                    np.pi,
                    ((-1, 0, 0, 0), (0, 1, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1)),
                ),
                (
                    "z",
                    np.pi,
                    ((-1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
                ),
            ]:
                self.assertEqual(mat, mod.rotate(axis, angle))

        def test_quat_mat(self):
            for quat, mat in [
                ((0, 0, 0, 1), np.identity(3)),
                ((0, 0, 1, 0), ((-1, 0, 0), (0, -1, 0), (0, 0, 1))),
                ((0, -0.5, 0, 0.5), ((0.5, 0, -0.5), (0, 1, 0), (0.5, 0, 0.5))),
            ]:
                self.assertEqual(mat, mod.quaternionToMatrix(quat))


if __name__ == "__main__":
    unittest.main()
