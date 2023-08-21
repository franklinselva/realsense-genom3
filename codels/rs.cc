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

#include <err.h>

/* --- Class realsense::camera -------------------------------------------- */
realsense::camera::camera()
{
    // Init sync structs with maximum sizes for frame queues
    v_sync.init(2); // color / FE left+right / IR left+right
    i_sync.init(1); // odometry / accelerometer / gyroscope
    d_sync.init(1); // pointcloud
}

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

void realsense::camera::clear_streams()
{
    _stream_desired.clear();
    _intr = rs2_intrinsics();
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
                    if (sp.stream_type() == RS2_STREAM_COLOR || sp.stream_type() == RS2_STREAM_FISHEYE)
                        _intr = sp.as<rs2::video_stream_profile>().get_intrinsics();
                }
            }
        }
        if (enabled.size() > 0)
        {
            s.open(enabled);

            // callback referenced in a C++11 lambda and passed to start()
            s.start([&](rs2::frame f)
                    { this->_callback(f); });
        }
    }
}

void realsense::camera::stop()
{
    for (rs2::sensor s : _sensors)
    {
        try
        {
            s.stop();
            s.close();
        }
        catch (rs2::error &e)
        {
            // warnx("rs error in stop: %s", e.what());
        } // catch exception for sensor not opened/streaming
    }
    _sensors.clear();
}

// Realsense frame callback function
void realsense::camera::_callback(rs2::frame f)
{
    if (f.is<rs2::depth_frame>())
    {
        std::unique_lock<std::mutex> lock(d_sync.m);
        d_sync.frames->enqueue(f);
        lock.unlock();
        d_sync.cv.notify_all();
    }
    else if (f.is<rs2::video_frame>())
    {
        std::unique_lock<std::mutex> lock(v_sync.m);

        // check stream type and fill struct accordingly
        rs2_stream st = f.get_profile().stream_type();
        // color and depth stream are composed of a single frame; retrieve it and notify
        if (st == RS2_STREAM_COLOR)
        {
            v_sync.frames->enqueue(f);
            lock.unlock();
            v_sync.cv.notify_all();
        }
        // enqueue FE frames and notify when both are retrieved
        else if (st == RS2_STREAM_FISHEYE)
        {
            v_sync.frames->enqueue(f);
            if (v_sync.frames->size() == 2)
            {
                lock.unlock();
                v_sync.cv.notify_all();
            }
        }
        // enqueue IR frames and notify when both are retrieved
        else if (st == RS2_STREAM_INFRARED)
        {
            v_sync.frames->enqueue(f);
            if (v_sync.frames->size() == 2)
            {
                lock.unlock();
                v_sync.cv.notify_all();
            }
        }
    }
    else if (f.is<rs2::pose_frame>() || (f.is<rs2::motion_frame>()))
    {
        std::unique_lock<std::mutex> lock(i_sync.m);
        i_sync.frames->enqueue(f);
        lock.unlock();
        i_sync.cv.notify_all();
    }
}
