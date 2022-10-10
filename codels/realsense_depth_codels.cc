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
#include <chrono>
#include <cmath>


/* --- Task depth ------------------------------------------------------- */


/** Codel rs_depth_start of task depth.
 *
 * Triggered by realsense_start.
 * Yields to realsense_sleep.
 */
genom_event
rs_depth_start(const realsense_pc *pc, const realsense_frame *frame,
               realsense_ids_range *depth_range, bool *registration,
               const genom_context self)
{
    pc->open("depth", self);
    (void) genom_sequence_reserve(&(pc->data("depth", self)->points), 0);
    (void) genom_sequence_reserve(&(pc->data("depth", self)->colors._value), 0);
    pc->data("depth", self)->colors._present = false;

    *registration = false;
    depth_range->min = 0.1;
    depth_range->max = 3.;

    frame->open("depth", self);
    (void) genom_sequence_reserve(&(frame->data("depth", self)->pixels), 0);

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
    while ((*d_sync)->_sync->frames->size())
    {
        (*d_sync)->_sync->frames->poll_for_frame(&f);
        (*d_data)->_data.enqueue(f);
    }

    lock.unlock();

    return realsense_main;
}


/** Codel rs_depth_main of task depth.
 *
 * Triggered by realsense_main.
 * Yields to realsense_img, realsense_sleep.
 */
genom_event
rs_depth_main(const or_camera_data *d_data, bool registration,
              const realsense_pc *pc, const genom_context self)
{
    rs2::frame f;
    d_data->_data.poll_for_frame(&f);

    rs2::pointcloud rs_pc;

    // // Map pc to color frame for registration
    // if (registration)
    //     rs_pc.map_to(rgb);

    // Get points and texture from frame
    rs2::points points = rs_pc.calculate(f);

    if (points.size())
    {
        // Update port data lenght and reallocate if need be
        or_sensor_pc* port_data = pc->data("depth", self);
        if (points.size() != port_data->points._length)
        {
            if (points.size() > port_data->points._maximum)
            {
                if (genom_sequence_reserve(&(port_data->points), points.size())  == -1)
                {
                    realsense_e_mem_detail d;
                    snprintf(d.what, sizeof(d.what), "unable to allocate 3d point memory");
                    return realsense_e_mem(&d,self);
                }
                // if (registration && genom_sequence_reserve(&(port_data->colors._value), points.size())  == -1)
                // {
                //         realsense_e_mem_detail d;
                //         snprintf(d.what, sizeof(d.what), "unable to allocate point color memory");
                //         return realsense_e_mem(&d,self);
                // }
                // else
                //     (void) genom_sequence_reserve(&(port_data->colors._value), 0);
            }
            port_data->points._length = points.size();
            // if (registration)
            // {
            //     port_data->colors._present = true;
            //     port_data->colors._value._length = points.size();
            // }
            // else
            //     port_data->colors._present = false;
        }

        // Copy data on port, update timestamp and write
        const rs2::vertex* vertices = points.get_vertices();
        // // Setup values needed to compute point colors from pixels
        // const rs2::texture_coordinate* text_coords = points.get_texture_coordinates();
        // uint16_t w = 0 , bpp = 0;
        // const uint8_t* color_data = NULL;
        // uint32_t x, y, p;
        // if (registration)
        // {
        //     w = rgb.as<rs2::video_frame>().get_width();
        //     bpp = rgb.as<rs2::video_frame>().get_bytes_per_pixel();
        //     color_data = (const uint8_t*) rgb.get_data();
        // }
        for (uint32_t i = 0; i < points.size(); i++)
        {
            port_data->points._buffer[i].x = vertices[i].x;
            port_data->points._buffer[i].y = vertices[i].y;
            port_data->points._buffer[i].z = vertices[i].z;

            // if (registration)
            // {
            //     x = static_cast<uint32_t>(text_coords[i].u * w);
            //     y = static_cast<uint32_t>(text_coords[i].v * w);
            //     p = (y * w + x) * bpp;
            //     memcpy(&port_data->colors._value._buffer[i], &color_data[p], bpp);
            //     // port_data->colors._value._buffer[i].r = color_data[p];
            //     // port_data->colors._value._buffer[i].g = color_data[p+1];
            //     // port_data->colors._value._buffer[i].b = color_data[p+2];
            // }
        }

        // Copy data on port, update timestamp and write
        double ms = f.get_timestamp();
        port_data->ts.sec = floor(ms/1000);
        port_data->ts.nsec = (ms - (double)port_data->ts.sec*1000) * 1e6;

        pc->write("depth", self);

        // Enqueue frame for 2D image generation in next codel
        d_data->_data.enqueue(f);

        return realsense_img;
    }

    return realsense_sleep;
}


/** Codel rs_depth_img of task depth.
 *
 * Triggered by realsense_img.
 * Yields to realsense_sleep.
 */
genom_event
rs_depth_img(const or_camera_data *d_data, bool registration,
             const realsense_ids_range *depth_range,
             const realsense_frame *frame, const genom_context self)
{
    rs2::frame f;
    d_data->_data.poll_for_frame(&f);

    or_sensor_frame* port_data = frame->data("depth", self);

    // Generate 2D image for display
    uint16_t w = f.as<rs2::video_frame>().get_width();
    uint16_t h = f.as<rs2::video_frame>().get_height();
    uint16_t bpp = 1; // grayscale only for now

    if (h*w*bpp != port_data->pixels._length)
    {
        if (h*w*bpp > port_data->pixels._maximum
            && genom_sequence_reserve(&(port_data->pixels), h*w*bpp)  == -1)
        {
            realsense_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate depth img frame memory");
            warnx("%s", d.what);
            return realsense_e_mem(&d,self);
        }
        port_data->pixels._length = h*w*bpp;
        port_data->height = h;
        port_data->width = w;
        port_data->bpp = bpp;
        port_data->compressed = false;
    }

    // memcpy(port_data->pixels._buffer, f.get_data(), port_data->pixels._length);
    rs2::depth_frame df = f.as<rs2::depth_frame>();
    for (uint16_t x=0; x<w; x++)
    for (uint16_t y=0; y<h; y++)
    {
        float d = max(min(df.get_distance(x,y), depth_range->max), depth_range->min);
        port_data->pixels._buffer[y*w+x] = static_cast<uint8_t>((d-depth_range->min)/(depth_range->max-depth_range->min)*255);
    }

    double ms = f.get_timestamp();
    port_data->ts.sec = floor(ms/1000);
    port_data->ts.nsec = (ms - (double)port_data->ts.sec*1000) * 1e6;

    frame->write("depth", self);

    return realsense_sleep;
}
