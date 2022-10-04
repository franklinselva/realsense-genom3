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


/* --- Task visual ------------------------------------------------------ */


/** Codel rs_viz_start of task visual.
 *
 * Triggered by realsense_start.
 * Yields to realsense_sleep.
 */
genom_event
rs_viz_start(realsense_ids *ids,
             const realsense_extrinsics *extrinsics,
             const realsense_frame *frame, const genom_context self)
{
    *extrinsics->data(self) = {0,0,0,0,0,0};
    extrinsics->write(self);

    ids->pipe = new or_camera_pipe();
    ids->v_sync = new realsense_sync_s();
    ids->i_sync = new realsense_sync_s();
    ids->pipe->v_sync = ids->v_sync;
    ids->pipe->i_sync = ids->i_sync;

    ids->v_data = new or_camera_data();
    ids->i_data = new or_camera_data();

    // ids->undist = new realsense_undist_s();
    // ids->info.compression_rate = -1;
    // ids->info.frequency = 30;
    //
    // frame->open("t265/1/raw", self);
    // frame->open("t265/1/compressed", self);
    // frame->open("t265/2/raw", self);
    // frame->open("t265/2/compressed", self);
    //
    // (void)genom_sequence_reserve(&(frame->data("t265/1/raw", self)->pixels), 0);
    // (void)genom_sequence_reserve(&(frame->data("t265/1/compressed", self)->pixels), 0);
    // (void)genom_sequence_reserve(&(frame->data("t265/2/raw", self)->pixels), 0);
    // (void)genom_sequence_reserve(&(frame->data("t265/2/compressed", self)->pixels), 0);

    return realsense_sleep;
}


/** Codel rs_viz_sleep of task visual.
 *
 * Triggered by realsense_sleep.
 * Yields to realsense_pause_sleep, realsense_poll.
 */
genom_event
rs_viz_sleep(bool started, const genom_context self)
{
    if (!started)
        return realsense_pause_sleep;
    else
        return realsense_poll;
}


/** Codel rs_viz_poll of task visual.
 *
 * Triggered by realsense_poll.
 * Yields to realsense_pause_poll, realsense_poll, realsense_main.
 */
genom_event
rs_viz_poll(realsense_sync_s **v_sync, or_camera_data **v_data,
            const genom_context self)
{
    std::unique_lock<std::mutex> lock((*v_sync)->_sync->m);

    (*v_sync)->_sync->cv.wait(lock);

    (*v_data)->_data = (*v_sync)->_sync->frame;

    lock.unlock();

    return realsense_main;
}


/** Codel rs_viz_main of task visual.
 *
 * Triggered by realsense_main.
 * Yields to realsense_sleep.
 */
genom_event
rs_viz_main(int16_t compression_rate, const or_camera_data *v_data,
            const realsense_undist_s *undist,
            const realsense_frame *frame, const genom_context self)
{
    rs2::frame f = v_data->_data;
    double ms = f.get_timestamp();
    long s = floor(ms/1000);
    long ns = (ms-s*1e3)*1e6;
    warnx("in: %s %ld.%09ld", f.get_profile().stream_name().c_str(), s, ns);


    // or_sensor_frame* rfdata = frame->data("t265/1/raw", self);
    //
    // rs2::video_frame rsframe = visual->data.get_fisheye_frame(1);
    // const uint16_t w = rsframe.get_width();
    // const uint16_t h = rsframe.get_height();
    // const double ms = rsframe.get_timestamp();
    //
    // // Mat cvframe = Mat(
    // //     Size(w, h),
    // //     CV_8UC1,
    // //     (void*) rsframe.get_data(),
    // //     Mat::AUTO_STEP
    // // );
    //
    // // remap(cvframe, cvframe, undist->m1, undist->m2, INTER_LINEAR);
    // //
    // // const uint32_t s = cvframe.size().height;
    //
    // if (w*h != rfdata->pixels._maximum)
    // {
    //
    //     if (genom_sequence_reserve(&(rfdata->pixels), w*h)  == -1) {
    //         realsense_e_mem_detail d;
    //         snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
    //         warnx("%s", d.what);
    //         return realsense_e_mem(&d,self);
    //     }
    //     rfdata->pixels._length = w*h;
    //     rfdata->height = h;
    //     rfdata->width = w;
    //     rfdata->bpp = 1;
    //     rfdata->compressed = false;
    // }
    //
    // memcpy(rfdata->pixels._buffer, (void*) rsframe.get_data(), rfdata->pixels._length);
    // rfdata->ts.sec = floor(ms/1000);
    // rfdata->ts.nsec = (ms - (double)rfdata->ts.sec*1000) * 1e6;
    //
    // frame->write("t265/1/raw", self);

    return realsense_sleep;
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
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    std::cout << devices.size() << std::endl;
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

    *started = true;

    // TMP enable streams manually here
    realsense::stream fe {RS2_STREAM_FISHEYE, RS2_FORMAT_Y8, 30, 848, 800};
    realsense::stream pose {RS2_STREAM_POSE, RS2_FORMAT_6DOF, 200, 0, 0};
    // realsense::stream accel {RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 62, 0,0};
    // realsense::stream gyro {RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200, 0,0};
    // realsense::stream accel {RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63, 0,0};
    // realsense::stream depth {RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 300, 848, 100};
    realsense::stream color {RS2_STREAM_COLOR, RS2_FORMAT_RGB8, 30, 1280, 720};

    (*pipe)->cam->add_stream(fe);
    (*pipe)->cam->add_stream(pose);
    // (*pipe)->cam->add_stream(accel);
    // (*pipe)->cam->add_stream(gyro);
    (*pipe)->cam->add_stream(color);
    // (*pipe)->cam->add_stream(depth);
    (*pipe)->cam->start();

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
    try {
        (*pipe)->cam->stop();
    } catch (rs2::error& e) {
        realsense_e_rs_detail d;
        snprintf(d.what, sizeof(d.what), "%s", e.what());
        warnx("rs error: %s", d.what);
        return realsense_e_rs(&d,self);
    }
    *started = false;

    warnx("disconnected from device");
    return realsense_ether;
}


/* --- Activity set_undistortion ---------------------------------------- */

/** Codel rs_set_undistort of activity set_undistortion.
 *
 * Triggered by realsense_start.
 * Yields to realsense_ether.
 * Throws realsense_e_io.
 */
genom_event
rs_set_undistort(uint16_t size, float fov, realsense_undist_s **undist,
                 const realsense_intrinsics *intrinsics,
                 const genom_context self)
{
    or_sensor_intrinsics* intr_data = intrinsics->data(self);

    // Get current calibration
    Mat K = Mat::zeros(3, 3, CV_32F);
    K.at<float>(0,0) = intr_data->calib.fx;
    K.at<float>(1,1) = intr_data->calib.fy;
    K.at<float>(0,2) = intr_data->calib.cx;
    K.at<float>(1,2) = intr_data->calib.cy;
    K.at<float>(0,1) = intr_data->calib.gamma;
    K.at<float>(2,2) = 1;
    Mat D = (Mat_<float>(4,1) <<
        intr_data->disto.k1,
        intr_data->disto.k2,
        intr_data->disto.k3,
        intr_data->disto.p1
    );

    // Compute desired calibration
    float f_px = size/2 /tan(fov/2);
    float c = size/2;
    Mat P = (Mat_<float>(3,3) <<
        f_px,    0, c,
           0, f_px, c,
           0,    0, 1
    );

    // Compute undistortion maps
    fisheye::initUndistortRectifyMap(K, D, Mat::eye(3,3, CV_32F), P, Size(size,size), CV_16SC2, (*undist)->m1, (*undist)->m2);

    // Publish intrinsincs with 'fake distortion' (=0) since the image is undistorted before publishing
    *intr_data = {
        .calib = { f_px, f_px, c, c, 0, },
        .disto = { 0, 0, 0, 0, 0, },
    };
    intrinsics->write(self);

    warnx("new undistortion maps computed");
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
