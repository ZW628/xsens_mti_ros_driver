# 使用说明

`xsens_ros_mti_driver`是`xsens`传感器的`ROS`驱动支持
该分支版本为`2022.0 Stable`,与`master`版本相比缺少了`status`话题的发布,修改后现已支持。

## 安装

在官网下载[Download MT Software Suite for Linux(x64)](https://www.movella.com/support/software-documentation)压缩后获取

``` bash
tar -xvf MT*.tar.gz

cd MT_Software_Suite_linux-x64_2022.0/
```

### xsens_ros_mti_driver

``` bash
./mtsdk_linux-x64_2022.0.sh

# `xsens_ros_mti_driver`文件夹即目标文件夹(ros包)

# 编译库文件 不同架构间不可移植
pushd xsens_ros_mti_driver/lib/xspublic && make && popd
```

### Xsens MT Manager

`gui`软件可对`Xsens`传感器进行配置，并可视化传感器数据，目前只支持 `Ubuntu 18.04`

``` bash
# 依赖 mtmanager/linux-x64/doc/MTM.README
apt -get install libxcb-xinerama0 libxcb-xinput0 libdouble-conversion1

# 解压
tar -xvf mtmanager_*.tar.gz

# 运行
./mtmanager/linux-x64/bin/mtmanager
```
