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

#include <deque>
#include <memory>
#include <iostream>
#include <mutex>
#include <chrono>
#include <thread>

#include "lumotive_ros/lumotive_client_interface.h"
#include "lumotive_ros/ylm_client.h"

#define MAX_QUEUE_SIZE		10

void pop_front_frame_unprotected(void);

static bool verbose = true;
static std::deque<std::shared_ptr<metadataFrame>> mFrame_stack;
static std::deque<std::shared_ptr<rangeFrame>> rFrame_stack;
static std::deque<std::shared_ptr<intensityFrame>> iFrame_stack;
static std::mutex mutex;

int read_frame(std::string IP, int port, uint32_t numPackets)
{
	// If numPackets = 0, then use automatic framing scheme
	// Otherwise, return when the requested number of packets has been read
	int ret;

	// TODO: Implement numPackets in common code
	ret = pollNetworkForFrame(IP, port);

	if (ret < 0) // If no data coming through, sleep to prevent needless resource consumption
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

	std::scoped_lock scoped_lock(mutex);
	if (ret > 0)
	{
		mFrame_stack.push_back(get_metadata_frame());
		rFrame_stack.push_back(get_range_frame());
		iFrame_stack.push_back(get_intensity_frame());
	}

	if (mFrame_stack.size() > MAX_QUEUE_SIZE)
	{
		if (verbose)
			std::cout << "Frame queue is maxed out!. Dropping oldest message." << std::endl;
		pop_front_frame_unprotected();
	}

	return ret;
}

bool is_frame_ready(void)
{
	std::scoped_lock scoped_lock(mutex);

	// I am making the assumption here that the idxFrame stack is fully in sync with the other stacks
	// So each stack is assumed to hold the same number of elements at all time
	return !mFrame_stack.empty();
}

void pop_front_frame(void)
{
	std::scoped_lock scoped_lock(mutex);
	pop_front_frame_unprotected();
}

void pop_front_frame_unprotected(void)
{
	mFrame_stack.pop_front();
	rFrame_stack.pop_front();
	iFrame_stack.pop_front();
}

std::shared_ptr<metadataFrame>& get_front_mFrame(void)
{
	std::scoped_lock scoped_lock(mutex);
	return mFrame_stack.front();
}

std::shared_ptr<rangeFrame>& get_front_rFrame(void)
{
	std::scoped_lock scoped_lock(mutex);
	return rFrame_stack.front();
}

std::shared_ptr<intensityFrame>& get_front_iFrame(void)
{
	std::scoped_lock scoped_lock(mutex);
	return iFrame_stack.front();
}

void set_verbose(bool val)
{
	verbose = val;
}

