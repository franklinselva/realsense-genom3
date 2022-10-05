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
#ifndef H_RS
#define H_RS

#include <condition_variable>
#include <mutex>

#include <librealsense2/rs.hpp>

#include <err.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;


namespace realsense
{
    struct stream
    {
        rs2_stream type;
        rs2_format format;
        int freq;
        int w;
        int h;
    };

    struct sync
    {
        std::vector<rs2::frame> frames;
        std::mutex              m;
        std::condition_variable cv;
    };

    class camera
    {
        private:
        std::vector<realsense::stream>  _stream_desired;
        std::vector<rs2::sensor>        _sensors;

        uint16_t _nb_depth_streams = 0; // number of depth stream to wait before notifying main thread
        std::vector<rs2::frame> _queue; // waiting queue for depth and fisheye frames

        void _callback(rs2::frame f);    // Callback function

        public:
        rs2_intrinsics  _intr;  // intrinsics calibration of video stream (FE / Color)

        realsense::sync v_sync;
        realsense::sync i_sync;

        ~camera();

        void init(rs2::device dev);

        void add_stream(realsense::stream s);
        void clear_streams();

        void start();
        void stop();
    };
}

#endif /* H_RS */
