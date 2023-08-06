
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

#include <ros/ros.h>
#include "xdainterface.h"

#include <iostream>
#include <stdexcept>
#include <string>

using std::chrono::milliseconds;

Journaller *gJournal = 0;


int main(int argc, char *argv[])
{
	//节点名称：xsens_driver
	ros::init(argc, argv, "xsens_driver");
	//ROS节点的主要接口，用于与ROS系统进行通信
	ros::NodeHandle node;

	//创建了一个XdaInterface类的对象xdaInterface,这是用于连接和管理Xsens MTi IMU设备的接口
	XdaInterface *xdaInterface = new XdaInterface();

	xdaInterface->registerPublishers(node);

	//如果连接失败，返回-1表示程序出错
	if (!xdaInterface->connectDevice())
		return -1;

	if (!xdaInterface->prepare())
		return -1;

	//进入一个无限循环，该循环会不断地调用xdaInterface对象的spinFor函数，用于获取IMU传感器数据
	while (ros::ok())
	{
		xdaInterface->spinFor(milliseconds(100));

		//调用ros::spinOnce()来处理ROS相关的回调函数和消息
		ros::spinOnce();
	}

	//释放xdaInterface对象的内存
	delete xdaInterface;

	return 0;
}
