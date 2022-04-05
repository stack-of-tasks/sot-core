import sys
import unittest
from os.path import abspath, dirname, join

from dynamic_graph.sot.core.parameter_server import ParameterServer
from example_robot_data import load_full

import parameter_server_conf as param_server_conf

param_server = ParameterServer("param_server")
param_server.init(0.001, "talos.urdf", "talos")

# Control time interval
dt = 0.001
robot_name = "robot"

urdfPath = param_server_conf.urdfFileName
urdfDir = param_server_conf.model_path


class TestParameterServer(unittest.TestCase):
    def test_set_parameter(self):
        # Read talos model
        path = join(dirname(dirname(abspath(__file__))), "models", "others", "python")
        sys.path.append(path)

        _, _, urdf_file_name, _ = load_full("talos")
        with open(urdf_file_name) as fs:
            urdf_rrbot_model_string = fs.read()

        param_server.setParameter("/robot_description", urdf_rrbot_model_string)
        model2_string = param_server.getParameter("/robot_description")

        self.assertEqual(urdf_rrbot_model_string, model2_string)

        aValue = 0.122
        param_server.setParameterDbl("/specificities/feet/right/size/height", aValue)
        a2Value = param_server.getParameterDbl("/specificities/feet/right/size/height")
        self.assertEqual(aValue, a2Value)


if __name__ == "__main__":
    unittest.main()
