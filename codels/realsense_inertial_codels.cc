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

#include "codels.hh"

#include <opencv2/opencv.hpp>
using namespace cv;

#include <err.h>
#include <cmath>
#include <iostream>


/* --- Task inertial ---------------------------------------------------- */


/** Codel rs_inertial_start of task inertial.
 *
 * Triggered by realsense_start.
 * Yields to realsense_sleep.
 */
genom_event
rs_inertial_start(const genom_context self)
{
    return realsense_sleep;
}


/** Codel rs_inertial_sleep of task inertial.
 *
 * Triggered by realsense_sleep.
 * Yields to realsense_pause_sleep, realsense_poll.
 */
genom_event
rs_inertial_sleep(bool started, const genom_context self)
{
    if (!started)
        return realsense_pause_sleep;
    else
        return realsense_poll;
}


/** Codel rs_inertial_poll of task inertial.
 *
 * Triggered by realsense_poll.
 * Yields to realsense_pause_poll, realsense_poll, realsense_main.
 */
genom_event
rs_inertial_poll(realsense_sync_s **i_sync, or_camera_data **i_data,
                 const genom_context self)
{
    std::unique_lock<std::mutex> lock((*i_sync)->_sync->m);

    (*i_sync)->_sync->cv.wait(lock);

    (*i_data)->_data = (*i_sync)->_sync->frame;

    lock.unlock();

    return realsense_main;
}


/** Codel rs_inertial_main of task inertial.
 *
 * Triggered by realsense_main.
 * Yields to realsense_sleep.
 */
genom_event
rs_inertial_main(int16_t compression_rate,
                 const or_camera_data *i_data,
                 const realsense_undist_s *undist,
                 const realsense_frame *frame,
                 const genom_context self)
{
    const rs2::frame* f = &i_data->_data;
    double ms = f->get_timestamp();
    long s = floor(ms/1000);
    long ns = (ms-s*1e3)*1e6;
    warnx("in: %s %ld.%09ld", f->get_profile().stream_name().c_str(), s, ns);

    return realsense_sleep;
}
