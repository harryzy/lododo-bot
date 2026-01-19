# IMU校准教程

[TOC]

## 1.说明

imu默认出厂已经校准，不需要再次校准。此校准主要是基于用户使用环境不同，导致imu不符合使用环境或者IMU数据异常时参考。进行校准前，**确认已经安装了驱动库，并且可以识别imu的设备**。

## 2.传输文件

将**5.源码文件**下的**imu校准**文件夹上传到自己使用的系统。这里以提供的ubuntu20.04的虚拟机镜像为例

![image-20251027175333286](image-20251027175333286.png) 

## 3.校准imu

把imu模块放平，运行YbImu_Calibrate_IMU.py程序，此时imu上有指示灯闪烁，确保校准过程稳定不动，等指示灯不在闪烁，就代表校准完成。

```
cd ~/IMU_calibration
```

```
python3 YbImu_Calibrate_IMU.py
```

![image-20251027175539536](image-20251027175539536.png)

## 4.校准磁力计（六轴版本无此功能）

```
cd ~/IMU_calibration
```

```
python3 YbImu_Calibrate_Mag.py
```

运行YbImu_Calibrate_Mag.py程序，此时imu上有指示灯闪烁，此时拿起模块在空中画八字校准。如果不知道如何操作，可以参考同目录下的视频。

## 5.校准温度（六轴版本、九轴版本无此功能）

```
cd ~/IMU_calibration
```

```
python3 YbImu_Calibrate_Temperature.py
```

运行YbImu_Calibrate_Temperature.py程序，此时程序会提示输入当前温度，根据自己目前环境的温度输入温度值，输入后按Enter键确认，温度校准完成。