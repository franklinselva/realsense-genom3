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

#include <err.h>
#include <cmath>
#include <sys/time.h>

#include <iostream>


/* --- Task depth ------------------------------------------------------- */


/** Codel rs_depth_start of task depth.
 *
 * Triggered by realsense_start.
 * Yields to realsense_sleep.
 */
genom_event
rs_depth_start(const realsense_pc *pc, const genom_context self)
{
    pc->open("depth", self);
    (void) genom_sequence_reserve(&(pc->data("depth", self)->points), 0);
    (void) genom_sequence_reserve(&(pc->data("depth", self)->colors._value), 0);
    pc->data("depth", self)->colors._present = false;

    return realsense_sleep;
}


/** Codel rs_depth_sleep of task depth.
 *
 * Triggered by realsense_sleep.
 * Yields to realsense_pause_sleep, realsense_poll.
 */
genom_event
rs_depth_sleep(bool started, const genom_context self)
{
    if (!started)
        return realsense_pause_sleep;
    else
        return realsense_poll;
}


/** Codel rs_depth_poll of task depth.
 *
 * Triggered by realsense_poll.
 * Yields to realsense_sleep, realsense_main.
 */
genom_event
rs_depth_poll(realsense_sync_s **d_sync, or_camera_data **d_data,
              const genom_context self)
{
    std::unique_lock<std::mutex> lock((*d_sync)->_sync->m);

    if (!(*d_sync)->_sync->frames->size())
        (*d_sync)->_sync->cv.wait_for(lock, std::chrono::duration<int16_t>(realsense_poll_duration_sec));

    if ((*d_sync)->_sync->frames->size() == 0)
        return realsense_sleep;

    rs2::frame f;
    (*d_sync)->_sync->frames->poll_for_frame(&f);
    (*d_data)->_data.enqueue(f);

    lock.unlock();

    return realsense_main;
}


/** Codel rs_depth_main of task depth.
 *
 * Triggered by realsense_main.
 * Yields to realsense_sleep.
 */
genom_event
rs_depth_main(const or_camera_data *d_data, bool registration,
              const realsense_pc *pc, const genom_context self)
{
    rs2::frame f;
    d_data->_data.poll_for_frame(&f);

    // Get points from frame
    rs2::pointcloud rs_pc;
    rs2::points points = rs_pc.calculate(f);

    // Map pc to color frame for registration
    // rs_pc.map_to(data->rgb);

    if (points.size())
    {
        // Update port data lenght and reallocate if need be
        or_sensor_pc* d_data = pc->data("depth", self);
        if (points.size() != d_data->points._length)
        {
            if (points.size() > d_data->points._maximum)
            {
                if (genom_sequence_reserve(&(d_data->points), points.size())  == -1)
                {
                    realsense_e_mem_detail d;
                    snprintf(d.what, sizeof(d.what), "unable to allocate 3d point memory");
                    return realsense_e_mem(&d,self);
                }
                // if (registration && genom_sequence_reserve(&(d_data->colors._value), points.size())  == -1)
                // {
                //         realsense_e_mem_detail d;
                //         snprintf(d.what, sizeof(d.what), "unable to allocate point color memory");
                //         return realsense_e_mem(&d,self);
                // }
                // else
                //     (void) genom_sequence_reserve(&(d_data->colors._value), 0);
            }
            d_data->points._length = points.size();
            // if (registration)
            // {
            //     d_data->colors._present = true;
            //     d_data->colors._value._length = points.size();
            // }
            // else
            //     d_data->colors._present = false;
        }

        // Copy data on port, update timestamp and write
        const rs2::vertex* vertices = points.get_vertices();
        // const rs2::texture_coordinate* text_coords = points.get_texture_coordinates();
        for (uint32_t i = 0; i < points.size(); i++)
        {
            d_data->points._buffer[i].x = vertices[i].x;
            d_data->points._buffer[i].y = vertices[i].y;
            d_data->points._buffer[i].z = vertices[i].z;
        }

        // Copy data on port, update timestamp and write
        double ms = f.get_timestamp();
        d_data->ts.sec = floor(ms/1000);
        d_data->ts.nsec = (ms - (double)d_data->ts.sec*1000) * 1e6;

        pc->write("depth", self);
    }

    return realsense_sleep;
}
