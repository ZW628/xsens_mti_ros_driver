
//  Copyright (c) 2003-2022 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xdacallback.h"

#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsdatapacket.h>

//初始化了m_maxBufferSize成员变量，表示缓冲区的最大容量
XdaCallback::XdaCallback(size_t maxBufferSize)
	: m_maxBufferSize(maxBufferSize)
{
}

XdaCallback::~XdaCallback() throw()
{
}

// Returns empty packet on timeout
//该函数用于从缓冲区中获取最新的传感器数据包，并在超时时返回一个空数据包。
//函数的参数timeout指定等待数据包的超时时间
RosXsDataPacket XdaCallback::next(const std::chrono::milliseconds &timeout)
{
	RosXsDataPacket packet;

	//对缓冲区进行加锁，以确保线程安全访问
	std::unique_lock<std::mutex> lock(m_mutex);

	/*等待新的数据包到达,如果缓冲区不为空，就取出第一个数据包，然后从缓冲区中移除它
	wait_for用于等待指定的时间段，直到条件变量满足或超过等待时间
	当前线程在等待指定的时间段 timeout 内，或者条件变量 m_buffer 不为空时，会立即返回
	如果等待时间 timeout 到期而 m_buffer 仍然为空，函数会返回 false */
	if (m_condition.wait_for(lock, timeout, [&] { return !m_buffer.empty(); }))
	{
		assert(!m_buffer.empty());

		packet = m_buffer.front();
		m_buffer.pop_front();
	}

	return packet;
}

//在有新的传感器数据包可用时会调用的回调函数
void XdaCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket *packet)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	ros::Time now = ros::Time::now();

	assert(packet != 0);

	// Discard oldest packet if buffer full
	//检查缓冲区是否已满。如果缓冲区已满，就移除最旧的数据包
	if (m_buffer.size() == m_maxBufferSize)
	{
		m_buffer.pop_front();
	}

	// Push new packet
	//将新的传感器数据包以ROS时间戳的形式添加到缓冲区末尾
	m_buffer.push_back(RosXsDataPacket(now, *packet));

	// Manual unlocking is done before notifying, to avoid waking up
	// the waiting thread only to block again
	//函数解锁互斥量并调用m_condition.notify_one()来通知等待的线程有新的数据包到达
	lock.unlock();
	m_condition.notify_one();
}
