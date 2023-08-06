
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

#ifndef XDAINTERFACE_H
#define XDAINTERFACE_H

#include <ros/ros.h>

#include "xdacallback.h"
#include <xstypes/xsportinfo.h>

#include "chrono"

struct XsControl;
struct XsDevice;


class PacketCallback;

//用于控制和管理IMU设备，并在ROS中发布传感器数据
class XdaInterface
{
public:
	XdaInterface();
	~XdaInterface();

	//用于获取IMU传感器数据，并在一段时间内等待数据包的到达
	void spinFor(std::chrono::milliseconds timeout);
	//注册需要发布的传感器数据的回调函数
	void registerPublishers(ros::NodeHandle &node);

	//用于连接到IMU设备
	bool connectDevice();
	//用于对IMU设备进行配置和准备工作
	bool prepare();
	//用于关闭设备连接并清理资源
	void close();

private:
	//用于注册回调函数
	void registerCallback(PacketCallback *cb);
	//用于处理错误信息
	bool handleError(std::string error);

	//指向XsControl对象的指针，用于管理和控制IMU设备
	XsControl *m_control;
	//表示连接的IMU设备
	XsDevice *m_device;
	//表示设备连接的端口信息
	XsPortInfo m_port;
	//用于管理和处理接收到的传感器数据
	XdaCallback m_xdaCallback;
	//用于存储已注册的回调函数的列表
	std::list<PacketCallback *> m_callbacks;
};

#endif
