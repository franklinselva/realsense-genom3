/*
 * Copyright (c) 2019 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                             Martin Jacquet - September 2022
 */

#include "acrealsense.h"

#include "realsense_c_types.h"

#include <err.h>
#include <cstdio>
#include <iostream>

#include "codels.hh"


/* --- Attribute set_compression ---------------------------------------- */

/** Validation codel set_compression_rate of attribute set_compression.
 *
 * Returns genom_ok.
 * Throws realsense_e_io.
 */
genom_event
set_compression_rate(int16_t compression_rate,
                     const genom_context self)
{
    if (compression_rate >= -1 && compression_rate <= 100)
        return genom_ok;
    else
    {
        realsense_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "unallowed compression rate");
        warnx("io error: %s", d.what);
        return realsense_e_io(&d,self);
    }
    return genom_ok;
}


/* --- Function enable_streams_t265 ------------------------------------- */

/** Codel enable_streams_t265 of function enable_streams_t265.
 *
 * Returns genom_ok.
 */
genom_event
enable_streams_t265(bool fisheye, bool odometry, bool accelerometer,
                    bool gyroscope, or_camera_pipe **pipe,
                    const genom_context self)
{
    (*pipe)->cam->clear_streams();
    if (fisheye)
    {
        realsense::stream fe {RS2_STREAM_FISHEYE, RS2_FORMAT_Y8, 30, 848, 800};
        (*pipe)->cam->add_stream(fe);
    }
    if (odometry)
    {
        realsense::stream pose {RS2_STREAM_POSE, RS2_FORMAT_6DOF, 200, 0, 0};
        (*pipe)->cam->add_stream(pose);
    }
    if (accelerometer)
    {
        realsense::stream accel {RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 62, 0,0};
        (*pipe)->cam->add_stream(accel);
    }
    if (gyroscope)
    {
        realsense::stream gyro {RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200, 0,0};
        (*pipe)->cam->add_stream(gyro);
    }

    return genom_ok;
}


/* --- Function enable_streams_d435 ------------------------------------- */

/** Codel enable_streams_d435 of function enable_streams_d435.
 *
 * Returns genom_ok.
 */
genom_event
enable_streams_d435(bool color, bool depth, bool infrared,
                    bool accelerometer, bool gyroscope,
                    or_camera_pipe **pipe, const genom_context self)
{
    (*pipe)->cam->clear_streams();
    if (color)
    {
        realsense::stream color {RS2_STREAM_COLOR, RS2_FORMAT_RGB8, 30, 1280, 720};
        (*pipe)->cam->add_stream(color);
    }
    if (depth)
    {
        realsense::stream depth {RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 30, 848, 480};
        (*pipe)->cam->add_stream(depth);
    }
    if (infrared)
    {
        realsense::stream ir {RS2_STREAM_INFRARED, RS2_FORMAT_Y8, 30, 848, 480};
        (*pipe)->cam->add_stream(ir);
    }
    if (accelerometer)
    {
        realsense::stream accel {RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63, 0,0};
        (*pipe)->cam->add_stream(accel);
    }
    if (gyroscope)
    {
        realsense::stream gyro {RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200, 0,0};
        (*pipe)->cam->add_stream(gyro);
    }

    return genom_ok;
}
