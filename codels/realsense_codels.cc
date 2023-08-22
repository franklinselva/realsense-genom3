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
 *
 * Modified by: Selvakumar H S - August 2023
 */

#include "acrealsense.h"

#include "realsense_c_types.h"

#include "codels.hh"

#include <err.h>

/* --- Attribute set_jpeg ----------------------------------------------- */

/** Validation codel set_compression_rate of attribute set_jpeg.
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
        snprintf(d.what, sizeof(d.what), "compression rate not allowed: %d", compression_rate);
        warnx("io error: %s", d.what);
        return realsense_e_io(&d, self);
    }
    return genom_ok;
}

/* --- Attribute set_format --------------------------------------------- */

/** Validation codel set_format of attribute set_format.
 *
 * Returns genom_ok.
 * Throws realsense_e_io.
 */
genom_event
set_format(const char format[8], const genom_context self)
{
    if (!strcmp(format, "YUYV") ||
        !strcmp(format, "RGB8") ||
        !strcmp(format, "RGBA8") ||
        !strcmp(format, "BGR8") ||
        !strcmp(format, "BGRA8") ||
        !strcmp(format, "Y16"))
        return genom_ok;
    else
    {
        realsense_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "format not allowed: %s ", format);
        warnx("io error: %s", d.what);
        return realsense_e_io(&d, self);
    }
}

/* --- Attribute set_fps ------------------------------------------------ */

/** Validation codel set_fps of attribute set_fps.
 *
 * Returns genom_ok.
 * Throws realsense_e_io.
 */
genom_event
set_fps(uint16_t frequency, uint16_t w, const genom_context self)
{
    if (frequency == 60 && w >= 1280)
    {
        realsense_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "60Hz only allowed below 1280x720");
        warnx("io error: %s", d.what);
        return realsense_e_io(&d, self);
    }
    if (frequency != 6 &&
        frequency != 15 &&
        frequency != 30 &&
        (frequency != 60))
    {
        realsense_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "frequency not allowed: %d", frequency);
        warnx("io error: %s", d.what);
        return realsense_e_io(&d, self);
    }
    return genom_ok;
}

/* --- Attribute set_size ----------------------------------------------- */

/** Validation codel set_size of attribute set_size.
 *
 * Returns genom_ok.
 * Throws realsense_e_io.
 */
genom_event
set_size(const or_camera_info_size_s *size, const genom_context self)
{
    if ((size->w == 1920 && size->h == 1080) ||
        (size->w == 1280 && size->h == 720) ||
        (size->w == 960 && size->h == 540) ||
        (size->w == 848 && size->h == 480) ||
        (size->w == 640 && size->h == 480) ||
        (size->w == 640 && size->h == 360) ||
        (size->w == 424 && size->h == 240) ||
        (size->w == 320 && size->h == 240))
        return genom_ok;
    else
    {
        realsense_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "format not allowed: %dx%d", size->w, size->h);
        warnx("io error: %s", d.what);
        return realsense_e_io(&d, self);
    }
}

/* --- Function setup_stream_t265 --------------------------------------- */

/** Codel enable_streams_t265 of function setup_stream_t265.
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
        realsense::stream fe{RS2_STREAM_FISHEYE, RS2_FORMAT_Y8, 30, 848, 800};
        (*pipe)->cam->add_stream(fe);
    }
    if (odometry)
    {
        realsense::stream pose{RS2_STREAM_POSE, RS2_FORMAT_6DOF, 200, 0, 0};
        (*pipe)->cam->add_stream(pose);
    }
    if (accelerometer)
    {
        realsense::stream accel{RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 62, 0, 0};
        (*pipe)->cam->add_stream(accel);
    }
    if (gyroscope)
    {
        realsense::stream gyro{RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200, 0, 0};
        (*pipe)->cam->add_stream(gyro);
    }

    return genom_ok;
}

/* --- Function setup_stream_d435 --------------------------------------- */

/** Codel enable_streams_d435 of function setup_stream_d435.
 *
 * Returns genom_ok.
 */
genom_event
enable_streams_d435(const or_camera_info *info, bool color, bool depth,
                    bool infrared, bool accelerometer, bool gyroscope,
                    or_camera_pipe **pipe, const genom_context self)
{
    (*pipe)->cam->clear_streams();
    if (color)
    {
        rs2_format format = RS2_FORMAT_ANY;
        if (!strcmp(info->format, "YUYV"))
            format = RS2_FORMAT_YUYV;
        else if (!strcmp(info->format, "RGB8"))
            format = RS2_FORMAT_RGB8;
        else if (!strcmp(info->format, "RGBA8"))
            format = RS2_FORMAT_RGBA8;
        else if (!strcmp(info->format, "BGR8"))
            format = RS2_FORMAT_BGR8;
        else if (!strcmp(info->format, "BGRA8"))
            format = RS2_FORMAT_BGRA8;
        else if (!strcmp(info->format, "Y16"))
            format = RS2_FORMAT_Y16;
        realsense::stream color{RS2_STREAM_COLOR, format, info->frequency, info->size.w, info->size.h};
        (*pipe)->cam->add_stream(color);
    }
    if (depth)
    {
        realsense::stream depth{RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 30, 848, 480};
        (*pipe)->cam->add_stream(depth);
    }
    if (infrared)
    {
        realsense::stream ir{RS2_STREAM_INFRARED, RS2_FORMAT_Y8, 30, 848, 480};
        (*pipe)->cam->add_stream(ir);
    }
    if (accelerometer)
    {
        realsense::stream accel{RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63, 0, 0};
        (*pipe)->cam->add_stream(accel);
    }
    if (gyroscope)
    {
        realsense::stream gyro{RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200, 0, 0};
        (*pipe)->cam->add_stream(gyro);
    }

    return genom_ok;
}
