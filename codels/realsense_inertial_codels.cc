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


/* --- Task inertial ---------------------------------------------------- */


/** Codel rs_inertial_start of task inertial.
 *
 * Triggered by realsense_start.
 * Yields to realsense_sleep.
 */
genom_event
rs_inertial_start(const realsense_accel *accel,
                  const realsense_gyro *gyro,
                  const realsense_odom *odom,
                  const genom_context self)
{
    accel->data(self)->ts.sec = 0;
    accel->data(self)->ts.nsec = 0;
    accel->data(self)->intrinsic = true;
    accel->data(self)->pos._present = false;
    accel->data(self)->pos_cov._present = false;
    accel->data(self)->att._present = false;
    accel->data(self)->att_cov._present = false;
    accel->data(self)->att_pos_cov._present = false;
    accel->data(self)->vel._present = false;
    accel->data(self)->vel_cov._present = false;
    accel->data(self)->avel._present = false;
    accel->data(self)->avel_cov._present = false;
    accel->data(self)->acc._present = false;
    accel->data(self)->acc_cov._present = false;
    accel->data(self)->aacc._present = false;
    accel->data(self)->aacc_cov._present = false;

    gyro->data(self)->ts.sec = 0;
    gyro->data(self)->ts.nsec = 0;
    gyro->data(self)->intrinsic = true;
    gyro->data(self)->pos._present = false;
    gyro->data(self)->pos_cov._present = false;
    gyro->data(self)->att._present = false;
    gyro->data(self)->att_cov._present = false;
    gyro->data(self)->att_pos_cov._present = false;
    gyro->data(self)->vel._present = false;
    gyro->data(self)->vel_cov._present = false;
    gyro->data(self)->avel._present = false;
    gyro->data(self)->avel_cov._present = false;
    gyro->data(self)->acc._present = false;
    gyro->data(self)->acc_cov._present = false;
    gyro->data(self)->aacc._present = false;
    gyro->data(self)->aacc_cov._present = false;

    odom->data(self)->ts.sec = 0;
    odom->data(self)->ts.nsec = 0;
    odom->data(self)->intrinsic = true;
    odom->data(self)->pos._present = false;
    odom->data(self)->pos_cov._present = false;
    odom->data(self)->att._present = false;
    odom->data(self)->att_cov._present = false;
    odom->data(self)->att_pos_cov._present = false;
    odom->data(self)->vel._present = false;
    odom->data(self)->vel_cov._present = false;
    odom->data(self)->avel._present = false;
    odom->data(self)->avel_cov._present = false;
    odom->data(self)->acc._present = false;
    odom->data(self)->acc_cov._present = false;
    odom->data(self)->aacc._present = false;
    odom->data(self)->aacc_cov._present = false;

    accel->write(self);
    gyro->write(self);
    odom->write(self);

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
 * Yields to realsense_sleep, realsense_main.
 */
genom_event
rs_inertial_poll(realsense_sync_s **i_sync, or_camera_data **i_data,
                 const genom_context self)
{
    std::unique_lock<std::mutex> lock((*i_sync)->_sync->m);

    if (!(*i_sync)->_sync->frames->size())
        (*i_sync)->_sync->cv.wait_for(lock, std::chrono::duration<int16_t>(realsense_poll_duration_sec));

    if ((*i_sync)->_sync->frames->size() == 0)
        return realsense_sleep;

    rs2::frame f;
    (*i_sync)->_sync->frames->poll_for_frame(&f);
    (*i_data)->_data.enqueue(f);

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
                 const realsense_accel *accel,
                 const realsense_gyro *gyro,
                 const realsense_odom *odom, const genom_context self)
{
    rs2::frame f;
    i_data->_data.poll_for_frame(&f);
    double ms = f.get_timestamp();

    rs2_stream type = f.get_profile().stream_type();
    if (type == RS2_STREAM_ACCEL)
    {
        or_pose_estimator_state* port_data = accel->data(self);
        rs2_vector acc = f.as<rs2::motion_frame>().get_motion_data();

        port_data->ts.sec = floor(ms/1000);
        port_data->ts.nsec = (ms - (double)port_data->ts.sec*1e3) * 1e6;

        port_data->acc._present = true;
        port_data->acc._value.ax = acc.x;
        port_data->acc._value.ay = acc.y;
        port_data->acc._value.az = acc.z;

        accel->write(self);
    }
    else if (type == RS2_STREAM_GYRO)
    {
        or_pose_estimator_state* port_data = gyro->data(self);
        rs2_vector avel = f.as<rs2::motion_frame>().get_motion_data();

        port_data->ts.sec = floor(ms/1000);
        port_data->ts.nsec = (ms - (double)port_data->ts.sec*1e3) * 1e6;

        port_data->avel._present = true;
        port_data->avel._value.wx = avel.x;
        port_data->avel._value.wy = avel.y;
        port_data->avel._value.wz = avel.z;

        gyro->write(self);
    }
    else if (type == RS2_STREAM_POSE)
    {
        or_pose_estimator_state* port_data = odom->data(self);
        rs2_pose pose = f.as<rs2::pose_frame>().get_pose_data();
        // Uncertainty is provided by the T265 as two confidence level integers:
        // pose.mapper_confidence: Pose map confidence 0 - Failed, 1 - Low, 2 - Medium, 3 - High
        // pose.tracker_confidence: Pose confidence 0 - Failed, 1 - Low, 2 - Medium, 3 - High
        // Antonio Enrique was using a default covariance (eg 1e-2), scaled by 1e(3-confidence)
        // any solution would by welcome
        port_data->ts.sec = floor(ms/1000);
        port_data->ts.nsec = (ms - (double)port_data->ts.sec*1e3) * 1e6;

        port_data->pos._present = true;
        port_data->pos._value.x = pose.translation.x;
        port_data->pos._value.y = pose.translation.y;
        port_data->pos._value.z = pose.translation.z;

        port_data->att._present = true;
        port_data->att._value.qw = pose.rotation.w;
        port_data->att._value.qx = pose.rotation.x;
        port_data->att._value.qy = pose.rotation.y;
        port_data->att._value.qz = pose.rotation.z;

        port_data->vel._present = true;
        port_data->vel._value.vx = pose.velocity.x;
        port_data->vel._value.vy = pose.velocity.y;
        port_data->vel._value.vz = pose.velocity.z;

        port_data->avel._present = true;
        port_data->avel._value.wx = pose.angular_velocity.x;
        port_data->avel._value.wy = pose.angular_velocity.y;
        port_data->avel._value.wz = pose.angular_velocity.z;

        port_data->acc._present = true;
        port_data->acc._value.ax = pose.acceleration.x;
        port_data->acc._value.ay = pose.acceleration.y;
        port_data->acc._value.az = pose.acceleration.z;

        port_data->aacc._present = true;
        port_data->aacc._value.awx = pose.angular_acceleration.x;
        port_data->aacc._value.awy = pose.angular_acceleration.y;
        port_data->aacc._value.awz = pose.angular_acceleration.z;

        odom->write(self);
    }

    return realsense_sleep;
}
