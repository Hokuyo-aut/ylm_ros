// Copyright (C) 2021-2022 Lumotive
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

#include <signal.h>

#include "lumotive_ros/lumotive_cloud_creation.h"
#include "lumotive_ros/network_handler.h"


void sig_handler(int sig_num)
{
    ros::shutdown();
    exit(sig_num);
}


/** @brief lumotive_driver node to capture Lumotive UDP packets,
 *         convert them in LumotiveRawPacket and publish them as
 *         LumotiveRawArray.
 */
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "lumotive_driver");

    ros::NodeHandle node;
    ros::NodeHandle param_nh("~");

    // Fetch port
    int port;
    param_nh.getParam("sensor_port", port);

    // Define signal
    signal(SIGINT, sig_handler);

    // Pointcloud handling
    lumotive_pointcloud::LumotiveFrameCreator pt(&node, &param_nh);

    // Packet Handling
    lumotive_network::NetworkHandler drv(&node, &param_nh);

    // Start threads
    ros::AsyncSpinner s(2);
    s.start();
    ros::waitForShutdown();

    return 0;
}
