// Copyright (C) 2021-2022 Lumotive
// Copyright 2023 HOKUYO AUTOMATIC CO.,LTD.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LUMOTIVE_CLOUD_CREATION_H
#define LUMOTIVE_CLOUD_CREATION_H

#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_field_conversion.h>

#include "lumotive_ros/lumotive_client_interface.h"
#include "lumotive_ros/lumotive_colors.h"
#include "lumotive_ros/lumotive_systems.h"


namespace lumotive_pointcloud {        
    

#define     EMPTY_MESSAGE_TIMER_SECS             5


/** @brief Class which converts Lumotive raw packets into sensor_msgs::PointCloud2 messages.map
 *         Framing can be done automatically using Lumotive's UDP stream framing logic, which
 *         relies on packet sequence numbers, or using a user-defined number of packets per
 *         frame. This class supports the creation of unorganized and organized PointCloud2 messages.
 *         For organized clouds, an RGB field is appended to each message with a range encoding. The
 *         messages can thus be directly converted to sensor_msgs::Image messages.
 */
class LumotiveFrameCreator
{
public:
    LumotiveFrameCreator(ros::NodeHandle *nh, ros::NodeHandle *param_nh);
    ~LumotiveFrameCreator(void);

private:
    bool organized_cloud_;
    bool reflectivity_flag_;
    float min_range_;
    float max_range_;
    float min_r_;
    float max_r_;
    std::string tf_frame_;
    float color_range_max_;
    const uint8_t (*colormap_in_use_)[3];
    const uint32_t *colormap_in_use_len_;

    ros::Timer pointcloud_publisher_timer_;
    ros::Timer empty_pointcloud_timer_;
    ros::Publisher lumotive_pointcloud_publisher_;
    void convert_lumotive_frame_to_pointcloud2(sensor_msgs::PointCloud2 *msg);
    void pointcloud_frame_ready_callback(const ros::TimerEvent&);
    void pointcloud_empty_msg_callback(const ros::TimerEvent&);
    color encode_range_to_RGB_with_colormap(float range_in_meters);
    color encode_range_to_RGB(float range_in_meters);
    bool is_range_valid(float range);
    void encode_color_by_reflectivity_fused_with_range_colormap(color *color_coded_range, float intensity, float range);
    void encode_reflectivity_to_brightness(color *color_coded_range, float intensity, float range);
};


}

#endif
