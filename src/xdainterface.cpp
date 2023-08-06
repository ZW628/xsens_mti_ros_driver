
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

#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"
#include "messagepublishers/statuspublisher.h"


//该类用于连接和管理Xsens MTi IMU设备，并在ROS中发布传感器数据
XdaInterface::XdaInterface()
	: m_device(nullptr)
{
	ROS_INFO("Creating XsControl object...");
	m_control = XsControl::construct();
	//确保m_control不为空
	assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
	ROS_INFO("Cleaning up ...");
	close();
	m_control->destruct();
}


void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	//获取传感器数据包
	//从XdaCallback m_xdaCallback中获取传感器数据包(XsDataPacket)，并将其存储到RosXsDataPacket类型的变量rosPacket中
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

	//获取到了非空的传感器数据包rosPacket.second，就遍历所有已注册的回调函数m_callbacks并执行它们，
	//将传感器数据和对应的时间戳传递给回调函数
	if (!rosPacket.second.empty())
	{
		/*在 for (auto &cb : m_callbacks) 循环中，cb 是一个指向 PacketCallback 对象的指针变量，
		它依次指向 m_callbacks 链表中的每个元素（即 PacketCallback 对象）。
		循环会依次执行循环体中的代码，每次都使用不同的 cb 指针来执行相应的操作。*/
		for (auto &cb : m_callbacks)
		{
			//调用每个回调函数来处理传感器数据包和时间戳
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

//根据ROS参数的配置，判断是否需要发布各种类型的传感器数据
//如果需要发布某类型的数据，就调用registerCallback函数来注册相应的发布器
void XdaInterface::registerPublishers(ros::NodeHandle &node)
{
	bool should_publish;

	if (ros::param::get("~pub_imu", should_publish) && should_publish)
	{
		registerCallback(new ImuPublisher(node));
	}
	if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(new OrientationPublisher(node));
	}
	if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(node));
	}
	if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(node));
	}
	if (ros::param::get("~pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(node));
	}
	if (ros::param::get("~pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(node));
	}
	if (ros::param::get("~pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(node));
	}
	if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(node));
	}
	if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(node));
	}
	if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(node));
	}
	if (ros::param::get("~pub_gnss", should_publish) && should_publish)
	{
		registerCallback(new GnssPublisher(node));
	}
	if (ros::param::get("~pub_twist", should_publish) && should_publish)
	{
		registerCallback(new TwistPublisher(node));
	}
	if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(new FreeAccelerationPublisher(node));
	}
	if (ros::param::get("~pub_transform", should_publish) && should_publish)
	{
		registerCallback(new TransformPublisher(node));
	}
	if (ros::param::get("~pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(new PositionLLAPublisher(node));
	}
	if (ros::param::get("~pub_velocity", should_publish) && should_publish)
	{
		registerCallback(new VelocityPublisher(node));
	}
	if(ros::param::get("~pub_status",should_publish) && should_publish){
		registerCallback(new StatusPublisher(node));
	}
}

//首先读取并解析ROS参数，如波特率、设备ID和端口名称。
//然后根据这些参数，使用XsScanner库来扫描设备端口并连接到IMU设备。
//如果成功连接到设备，则在日志中打印设备信息，并将设备的回调处理器m_xdaCallback添加到设备上。
bool XdaInterface::connectDevice()
{
	// Read baudrate parameter if set
	XsBaudRate baudrate = XBR_Invalid;
	if (ros::param::has("~baudrate"))
	{
		int baudrateParam = 0;
		ros::param::get("~baudrate", baudrateParam);
		ROS_INFO("Found baudrate parameter: %d", baudrateParam);
		baudrate = XsBaud::numericToRate(baudrateParam);
	}
	// Read device ID parameter
	bool checkDeviceID = false;
	std::string deviceId;
	if (ros::param::has("~device_id"))
	{
		ros::param::get("~device_id", deviceId);
		checkDeviceID = true;
		ROS_INFO("Found device ID parameter: %s.",deviceId.c_str());

	}
	// Read port parameter if set
	XsPortInfo mtPort;
	if (ros::param::has("~port"))
	{
		std::string portName;
		ros::param::get("~port", portName);
		ROS_INFO("Found port name parameter: %s", portName.c_str());
		mtPort = XsPortInfo(portName, baudrate);
		ROS_INFO("Scanning port %s ...", portName.c_str());
		if (!XsScanner::scanPort(mtPort, baudrate))
			return handleError("No MTi device found. Verify port and baudrate.");
		if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
			return handleError("No MTi device found with matching device ID.");

	}
	else
	{
		ROS_INFO("Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found.");

	ROS_INFO("Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	ROS_INFO("Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	ROS_INFO("Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::prepare()
{
	//断言m_device不为空
	assert(m_device != 0);

	//将设备切换到配置模式，并读取配置文件中存储的设备配置
	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	ROS_INFO("Measuring ...");
	//将设备切换到测量模式，准备开始测量
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	std::string logFile;
	//如果在ROS参数中设置了日志文件名log_file，则函数将创建该日志文件，并开始记录传感器数据
	if (ros::param::get("~log_file", logFile))
	{
		if (m_device->createLogFile(logFile) != XRV_OK)
			return handleError("Failed to create a log file! (" + logFile + ")");
		else
			ROS_INFO("Created a log file: %s", logFile.c_str());

		ROS_INFO("Recording to %s ...", logFile.c_str());
		if (!m_device->startRecording())
			return handleError("Could not start recording");
	}

	return true;
}

void XdaInterface::close()
{
	//判断设备是否为空指针
	if (m_device != nullptr)
	{
		//停止设备的记录功能，并关闭设备日志文件
		m_device->stopRecording();
		m_device->closeLogFile();
		//将m_xdaCallback从设备中移除
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	//关闭设备的端口
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	//将传入的回调函数指针cb添加到m_callbacks列表中，表示该回调函数已注册
	m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
	//将错误信息打印到ROS日志中，然后调用close()函数来关闭设备连接和清理资源，最后返回false表示处理错误失败
	ROS_ERROR("%s", error.c_str());
	close();
	return false;
}
