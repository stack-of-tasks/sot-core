import unittest
import pinocchio as pin
import parameter_server_conf as param_server_conf
from dynamic_graph.sot.core.parameter_server import ParameterServer
import sys
from os.path import dirname, join, abspath

# Switch pinocchio to numpy matrix
pin.switchToNumpyMatrix()

param_server = ParameterServer("param_server")
param_server.init(0.001, "talos.urdf", "talos")

# Control time interval
dt = 0.001
robot_name = 'robot'

urdfPath = param_server_conf.urdfFileName
urdfDir = param_server_conf.model_path


class TestParameterServer(unittest.TestCase):
    def test_set_parameter(self):
        # Read talos model
        path = join(dirname(dirname(abspath(__file__))), 'models', 'others', 'python')
        sys.path.append(path)

        from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR

        urdf_file_name = EXAMPLE_ROBOT_DATA_MODEL_DIR + \
            '/talos_data/robots/talos_reduced.urdf'

        fs = open(urdf_file_name, 'r')
        urdf_rrbot_model_string = fs.read()
        fs.close()

        param_server.setParameter("/robot_description", urdf_rrbot_model_string)
        model2_string = param_server.getParameter("/robot_description")

        self.assertEqual(urdf_rrbot_model_string, model2_string)

        aValue = 0.122
        param_server.setParameterDbl("/specificities/feet/right/size/height", aValue)
        a2Value = param_server.getParameterDbl("/specificities/feet/right/size/height")
        self.assertEqual(aValue, a2Value)


if __name__ == '__main__':
    unittest.main()
