import unittest
import numpy as np
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox

# Switch pinocchio to numpy matrix
pin.switchToNumpyMatrix()

import parameter_server_conf as param_server_conf
from dynamic_graph.sot.core.parameter_server import ParameterServer

param_server = ParameterServer("param_server")
param_server.init(0.001, "talos.urdf", "talos")

# Control time interval
dt = 0.001
robot_name = 'robot'

urdfPath = param_server_conf.urdfFileName
urdfDir = param_server_conf.model_path

import sys

from os.path import dirname, join, abspath


class TestParameterServer(unittest.TestCase):
    def test_set_parameter(self):
        # Read talos model
        path = join(
            dirname(dirname(abspath(__file__))), 'models', 'others', 'python')
        sys.path.append(path)

        from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR

        urdf_file_name=EXAMPLE_ROBOT_DATA_MODEL_DIR+\
           '/talos_data/robots/talos_reduced.urdf'

        fs = open(urdf_file_name, 'r')
        urdf_rrbot_model_string = fs.read()
        fs.close()

        print(EXAMPLE_ROBOT_DATA_MODEL_DIR)
        #
        model = pin.buildModelFromXML(urdf_rrbot_model_string)

        neutral_configuration_model = pin.neutral(model)
        jointIds = []
        #print(model)
        reduced_model = pin.buildReducedModel(model, jointIds,
                                              neutral_configuration_model)

        data = model.createData()
        com = pin.centerOfMass(model, data, neutral_configuration_model)
        pin.updateFramePlacements(model, data)
        m = data.mass[0]
        param_server.setParameter("urdf_model", urdf_rrbot_model_string)
        model2_string = param_server.getParameter("urdf_model")

        self.assertEqual(urdf_rrbot_model_string, model2_string)


if __name__ == '__main__':
    unittest.main()
