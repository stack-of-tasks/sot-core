#!/usr/bin/env python

import unittest


class PythonImportTest(unittest.TestCase):
    def test_math_small_entities(self):
        try:
            import dynamic_graph.sot.core.math_small_entities  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_feature_position_relative(self):
        try:
            import dynamic_graph.sot.core.feature_position_relative  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_feature_position(self):
        try:
            import dynamic_graph.sot.core.feature_position  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_matrix_util(self):
        try:
            import dynamic_graph.sot.core.matrix_util  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_meta_task_6d(self):
        try:
            import dynamic_graph.sot.core.meta_task_6d  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_meta_task_posture(self):
        try:
            import dynamic_graph.sot.core.meta_task_posture  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_meta_task_visual_point(self):
        try:
            import dynamic_graph.sot.core.meta_task_visual_point  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_meta_tasks_kine_relative(self):
        try:
            import dynamic_graph.sot.core.meta_tasks_kine_relative  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_meta_tasks_kine(self):
        try:
            import dynamic_graph.sot.core.meta_tasks_kine  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_meta_tasks(self):
        try:
            import dynamic_graph.sot.core.meta_tasks  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_attime(self):
        try:
            import dynamic_graph.sot.core.utils.attime  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_history(self):
        try:
            import dynamic_graph.sot.core.utils.history  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_thread_interruptible_loop(self):
        try:
            import dynamic_graph.sot.core.utils.thread_interruptible_loop  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_viewer_helper(self):
        try:
            import dynamic_graph.sot.core.utils.viewer_helper  # noqa
        except ImportError as ie:
            self.fail(str(ie))

    def test_viewer_loger(self):
        try:
            import dynamic_graph.sot.core.utils.viewer_loger  # noqa
        except ImportError as ie:
            self.fail(str(ie))


if __name__ == '__main__':
    unittest.main()
