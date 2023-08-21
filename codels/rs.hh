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
        rs2::frame_queue *frames;
        std::mutex m;
        std::condition_variable cv;

        void init(uint16_t size) { frames = new rs2::frame_queue(size, true); }
        ~sync() { delete frames; }
    };

    class camera
    {
    private:
        std::vector<realsense::stream> _stream_desired;
        std::vector<rs2::sensor> _sensors;

        void _callback(rs2::frame f); // callback function

    public:
        rs2_intrinsics _intr; // intrinsics calibration of video stream (FE / Color)

        realsense::sync v_sync;
        realsense::sync i_sync;
        realsense::sync d_sync;

        camera();
        ~camera();

        void init(rs2::device dev);

        void add_stream(realsense::stream s);
        void clear_streams();

        void start();
        void stop();
    };
}

#endif /* H_RS */
