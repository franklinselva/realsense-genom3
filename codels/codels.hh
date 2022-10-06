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
#ifndef H_RS_CODELS
#define H_RS_CODELS

#include "acrealsense.h"

#include "realsense_c_types.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "rs.hh"


struct or_camera_data {
    rs2::frame_queue _data = rs2::frame_queue(2, true);
};

struct realsense_sync_s {
    realsense::sync* _sync;
};

struct or_camera_pipe {
    realsense::camera* cam;

    or_camera_pipe() { cam = new realsense::camera(); }
    ~or_camera_pipe() { delete cam; }

    void init(rs2::device dev) { cam->init(dev); }
};

struct realsense_undist_s {
    bool enabled;
    Mat m1, m2;
};

#endif /* H_RS_CODELS */
