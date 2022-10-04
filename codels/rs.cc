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
#include "rs.hh"

#include <condition_variable>
#include <mutex>

#include <librealsense2/rs.hpp>

#include <err.h>
#include <iostream>
using namespace cv;


realsense::camera::~camera() { this->stop(); }

void realsense::camera::init(rs2::device dev)
{
    if (_sensors.size() == 0)
    {
        _sensors = dev.query_sensors();
    }
}

void realsense::camera::add_stream(realsense::stream s)
{
    _stream_desired.push_back(s);
}

void realsense::camera::rm_stream(realsense::stream s)
{
    // TODO
}

void realsense::camera::start()
{
    for (rs2::sensor s : _sensors)
    {
        std::vector<rs2::stream_profile> enabled;
        for (rs2::stream_profile sp : s.get_stream_profiles())
        {
            for (realsense::stream stream_d : _stream_desired)
            {
                if (sp.stream_type() == stream_d.type &&
                    sp.format() == stream_d.format &&
                    sp.fps() == stream_d.freq &&
                    (!sp.is<rs2::video_stream_profile>() ||
                    (sp.as<rs2::video_stream_profile>().width() == stream_d.w &&
                    sp.as<rs2::video_stream_profile>().height() == stream_d.h)))
                {
                    enabled.push_back(sp);
                }
            }
        }
        if (enabled.size() > 0)
        {
            s.open(enabled);

            // callback referenced in a C++11 lambda and passed to start()
            s.start([&](rs2::frame f) { this->_callback(f); });
        }
    }
}

void realsense::camera::stop()
{
    for (rs2::sensor s : _sensors)
    {
        s.stop();
        s.close();
    }
    _sensors.clear();
}

// Callback function
void realsense::camera::_callback(rs2::frame f)
{
    rs2::stream_profile sp = f.get_profile();
    if (sp.stream_type() == RS2_STREAM_FISHEYE ||
        sp.stream_type() == RS2_STREAM_COLOR)
    {
        std::unique_lock<std::mutex> lock(v_sync.m);
        v_sync.frame = f;
        lock.unlock();
        v_sync.cv.notify_all();
    }
    else if (sp.stream_type() == RS2_STREAM_POSE ||
        sp.stream_type() == RS2_STREAM_ACCEL ||
        sp.stream_type() == RS2_STREAM_GYRO)
    {
        std::unique_lock<std::mutex> lock(i_sync.m);
        i_sync.frame = f;
        lock.unlock();
        i_sync.cv.notify_all();
    }
}
