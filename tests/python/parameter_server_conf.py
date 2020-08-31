NJ = 2
model_path = ['/integration_tests/openrobots/share/']
urdfFileName = "/integration_tests/openrobots/share/" + \
    "simple_humanoid_description/urdf/" + \
    "simple_humanoid.urdf"
ImuJointName = "imu_joint"

mapJointNameToID = {
    'j1': 0,
    'j2': 1,
}

mapJointLimits = {
    0: [-0.349065850399, 1.57079632679],
    1: [-0.5236, 0.5236],
}

vfMax = [100.0, 100.0]
vfMin = [-100.0, -100.0]

mapForceIdToForceLimits = {0: [vfMin, vfMax], 1: [vfMin, vfMax]}

mapNameToForceId = {"rf": 0, "lf": 1}

indexOfForceSensors = ()

footFrameNames = {"Right": "RLEG_ANKLE_R", "Left": "LLEG_ANKLE_R"}

rightFootSensorXYZ = (0.0, 0.0, -0.085)
rightFootSoleXYZ = (0.0, 0.0, -0.105)

urdftosot = (0, 1)
