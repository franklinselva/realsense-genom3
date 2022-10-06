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


/* --- Task main -------------------------------------------------------- */


/** Codel rs_start of task main.
 *
 * Triggered by realsense_start.
 * Yields to realsense_ether.
 */
genom_event
rs_start(realsense_ids *ids, const realsense_extrinsics *extrinsics,
         const realsense_intrinsics *intrinsics,
         const realsense_frame *frame, const realsense_pc *pc,
         const genom_context self)
{
    *extrinsics->data(self) = {0,0,0,0,0,0};
    extrinsics->write(self);
    *intrinsics->data(self) = {
        .calib = {0, 0, 0, 0, 0},
        .disto = {0, 0, 0, 0, 0},
    };
    intrinsics->write(self);

    ids->pipe = new or_camera_pipe();
    ids->v_sync = new realsense_sync_s();
    ids->i_sync = new realsense_sync_s();
    ids->d_sync = new realsense_sync_s();
    ids->v_sync->_sync = &ids->pipe->cam->v_sync;
    ids->i_sync->_sync = &ids->pipe->cam->i_sync;
    ids->d_sync->_sync = &ids->pipe->cam->d_sync;

    ids->v_data = new or_camera_data();
    ids->i_data = new or_camera_data();
    ids->d_data = new or_camera_data();

    ids->undist = new realsense_undist_s();
    ids->info.compression_rate = -1;
    ids->info.frequency = 30;
    snprintf(ids->info.format, sizeof(ids->info.format), "RGB8");
    ids->info.size = {1280, 720};

    return realsense_ether;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel rs_connect of activity connect.
 *
 * Triggered by realsense_start.
 * Yields to realsense_ether.
 * Throws realsense_e_rs, realsense_e_io.
 */
genom_event
rs_connect(const char serial[32], or_camera_pipe **pipe, bool *started,
           const realsense_intrinsics *intrinsics,
           const genom_context self)
{
    // Stop current connexion
    (*pipe)->cam->stop();

    // Find device in list (default or by serial)
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();

    if (devices.size() == 0)
    {
        realsense_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "no rs device connected");
        warnx("%s", d.what);
        return realsense_e_io(&d,self);
    }

    rs2::device device_des;
    if (!strcmp(serial,"\0") || !strcmp(serial,"0"))
        device_des = devices[0];
    else
        for (uint16_t i = 0; i<devices.size(); i++)
        {
            if (!strcmp(serial, devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)))
            {
                device_des = devices[i];
                break;
            }
            if (i == devices.size()-1)
            {
                realsense_e_io_detail d;
                snprintf(d.what, sizeof(d.what), "rs device with serial %s not connected", serial);
                warnx("%s", d.what);
                return realsense_e_io(&d,self);
            }
        }

    (*pipe)->init(device_des);

    // Get device name and serial for warnx
    std::string name = "Unknown Device";
    if (device_des.supports(RS2_CAMERA_INFO_NAME))
        name = device_des.get_info(RS2_CAMERA_INFO_NAME);
    std::string sn = "###";
    if (device_des.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        sn = device_des.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    warnx("Connecting to device: %s #%s", name.c_str(), sn.c_str());

    // Start all desired streams
    (*pipe)->cam->start();

    // Init intrinsics port if COLOR or FISHEYE is enabled
    rs2_intrinsics* intr = &(*pipe)->cam->_intr;
    if (intr->width != 0)
    {
        *intrinsics->data(self) = {
            .calib = {
                intr->fx,
                intr->fy,
                intr->ppx,
                intr->ppy,
                0,
            },
            .disto = {
                intr->coeffs[0],
                intr->coeffs[1],
                intr->coeffs[2],
                intr->coeffs[3],
                intr->coeffs[4],
            },
        };
        intrinsics->write(self);
    }

    *started = true;

    return realsense_ether;

}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel rs_disconnect of activity disconnect.
 *
 * Triggered by realsense_start.
 * Yields to realsense_ether.
 * Throws realsense_e_rs.
 */
genom_event
rs_disconnect(or_camera_pipe **pipe, bool *started,
              const genom_context self)
{
    (*pipe)->cam->stop();
    *started = false;

    warnx("disconnected from device");
    return realsense_ether;
}


/* --- Activity set_extrinsics ------------------------------------------ */

/** Codel rs_set_extrinsics of activity set_extrinsics.
 *
 * Triggered by realsense_start.
 * Yields to realsense_ether.
 */
genom_event
rs_set_extrinsics(const sequence6_float *ext_values,
                  const realsense_extrinsics *extrinsics,
                  const genom_context self)
{
    *extrinsics->data(self) = {
        ext_values->_buffer[0],
        ext_values->_buffer[1],
        ext_values->_buffer[2],
        ext_values->_buffer[3],
        ext_values->_buffer[4],
        ext_values->_buffer[5],
    };
    extrinsics->write(self);

    warnx("new extrinsic calibration");
    return realsense_ether;
}
