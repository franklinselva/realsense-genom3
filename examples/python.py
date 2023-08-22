# This example shows how to use the python bindings for d435 realsense camera.

import genomix
import os

LIB_PATH = os.environ.get("DRONE_VV_PATH")
MODULE_PATH = os.path.join(LIB_PATH, "lib/genom/pocolibs/plugins/realsense.so")

handle = genomix.connect("localhost:8080")

camera = handle.load(MODULE_PATH)

# Setup streaming
camera.setup_stream_d435(
    color=True, depth=True, infrared=True, accelerometer=False, gyroscope=False
)

# Connect the camera
# This will connect the camera and start streaming
camera.connect_d435("832112070817")  # Serial number of the camera

# Camera Configurations
camera.set_format(format="RGB8")
camera.set_fps(frequency=30)
camera.set_size(size={"w": 640, "h": 480})

# Pointcloud configurations
camera.set_registration(True)
camera.set_depth_range(min=0.1, max=3.0)

# To get camera color image
image = camera.frame("color")
depth_image = camera.frame("depth")
infrared_left_image = camera.frame("IR_l")
infrared_right_image = camera.frame("IR_r")

# To get the camera intrinsics
intrinsics = camera.intrinsics()

# To get the camera extrinsics
extrinsics = camera.extrinsics()

# To get the pointcloud
pointcloud = camera.pc("depth")

# To get acceleration and gyroscope data
# accelerometer = camera.accel()
# gyroscope = camera.gyro()
