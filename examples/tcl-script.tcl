# This example shows how to use the python bindings for d435 realsense camera.
# This script is not tested but gives an idea of how to use the bindings.

package require genomix

set LIB_PATH $env(DRONE_VV_PATH)
set MODULE_PATH [file join $LIB_PATH "lib/genom/pocolibs/plugins/realsense.so"]

set handle [genomix connect localhost:8080]

set camera [$handle load $MODULE_PATH]

# Setup streaming
$camera setup_stream_d435 -color true -depth true -infrared true -accelerometer true -gyroscope true

# Connect the camera
# This will connect the camera and start streaming
$camera connect_d435 "832112070817" ;# Serial number of the camera

# Camera Configurations
$camera set_format -format "RGB8"
$camera set_fps -frequency 30
$camera set_size -size [list width 640 height 480]

# Pointcloud configurations
$camera set_registration true
$camera set_depth_range -min 0.1 -max 3.0

# To get camera color image
set image [$camera frame "color"]
set depth_image [$camera frame "depth"]
set infrared_left_image [$camera frame "IR_l"]
set infrared_right_image [$camera frame "IR_r"]

# To get the camera intrinsics
set intrinsics [$camera intrinsics]

# To get the camera extrinsics
set extrinsics [$camera extrinsics]

# To get the pointcloud
set pointcloud [$camera pc "depth"]

# To get acceleration and gyroscope data
set accelerometer [$camera accel]
set gyroscope [$camera gyro]