# 2025/01/07

> 今天主要就是进行工程环境的配置，
>
> 将ubuntu基础环境配置好以后，配置buildmap工程环境，报啥错就安装啥。
>
> 以下是各种软件的安装流程

## 雷达驱动安装

手动安装这俩货，然后替换掉cmakelists中的路径

[livox_ros2_driver](https://github.com/Livox-SDK/livox_ros2_driver)

```c++
set(livox_interfaces_DIR "/home/wenming/ws_livox/install/livox_interfaces/share/livox_interfaces/cmake/")
find_package(livox_interfaces REQUIRED)
```

[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

```c++
set(livox_ros_driver2_DIR "/home/wenming/livox_ros_driver2/install/livox_ros_driver2/share/livox_ros_driver2/cmake/")
find_package(livox_ros_driver2 REQUIRED)
```

## **GTSAM库安装**

[参考链接](https://blog.csdn.net/weixin_40324045/article/details/121284253)

查看boost版本

```c++
dpkg -s libboost-dev | grep Version
```

`gcc`版本

```c++
gcc --version
```

进行安装，源码地址：[gtsam](https://github.com/borglab/gtsam)

进入源码目录，创建`build`文件夹

```c++
cd xxx
mkdir build
cd build
```

编译

```c++
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_EIGEN_MKL=OFF -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
```

安装

```c++
sudo make install
```

## bumblebee

> 这是公司内部自己开发的，需要自己去Gitlab上下载

下的时候直接切换分支

```
git clone -b yichang_test_optimization http://192.168.0.12/iai-lab/autocrane/bumblebee.git
```

```c++
git branch
git checkout <分支>
```

**需要在buildmap中先修改`CMakeLists.txt`src那行注释掉先编译`buildmap`，再编译bumblebee，就能成功通过**

## glog

[安装步骤](https://blog.csdn.net/Cv_Ys/article/details/127327904)

## 使用雷达

![image-20250107162835762](assets/image-20250107162835762.png)

用的是`/LivoxViewer1$ ./livox_viewer.sh` ，没有用2

雷达的广播码为3JEDLBA0019V441

![image-20250107162938282](assets/image-20250107162938282.png)

## 运行buildmap

先启动雷达驱动

```c++
ros2 launch livox_ros2_driver livox_lidar_msg_launch.py
```

再启动建图节点

```c++
ros2 launch buildmap mapping.launch.py
```

> 启动时如果遇到类似如下问题![image-20250107175431917](assets/image-20250107175431917.png)
>
> 这个错误提示表明 `fastlio_mapping` 进程启动时无法加载 `liblivox_interfaces__rosidl_typesupport_cpp.so` 动态链接库
>
> 解决步骤：
>
> 1. 确认库文件是否存在
>
>    首先，确认是否已经安装了包含 `liblivox_interfaces__rosidl_typesupport_cpp.so` 的库文件。你可以使用 `find` 命令在系统中查找该文件：
>
>    ```c++
>    sudo find / -name "liblivox_interfaces__rosidl_typesupport_cpp.so"
>    ```
>
>    如果文件存在，记下文件所在的路径。比如，假设它位于 `/usr/local/lib`。
>
> 2. 检查 `LD_LIBRARY_PATH`
>
>    如果找到了库文件但仍然无法加载，可能是因为 `LD_LIBRARY_PATH` 环境变量没有正确设置。你需要将该库所在的目录添加到 `LD_LIBRARY_PATH` 中。
>
>    例如，如果库文件位于 `/usr/local/lib`，你可以运行：
>
>    ```c++
>    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
>    ```
>
>    为了确保每次启动终端时都能加载这个路径，可以将此命令添加到你的 shell 配置文件中（例如 `~/.bashrc` 或 `~/.zshrc`）：
>
>    ```c++
>    echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
>    source ~/.bashrc
>    ```

启动效果如下：

![4d0e51585ab8d393f7334b4641f659dd](assets/4d0e51585ab8d393f7334b4641f659dd.jpg)

# 2025/01/08

> 今天先做一下livox avia自带的imu的内外参。

通过读avia的使用手册可以知道，内置的IMU的型号为[BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)，

[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK/blob/master/README_CN.md)

## avia手册相关

由坐标关系可知外参矩阵![image-20250108090834389](assets/image-20250108090834389.png)

说明书的时间戳一节，解释了三种同步方式，IEEE 1588-2008，PPS，GPS 同步。

可以在Livox Viewer 或 SDK 进行配置，配置多回波模式、IMU数据推送频率。

[livox-ros2-driver驱动程序中文说明](https://github.com/Livox-SDK/livox_ros2_driver/blob/master/README_CN.md)

![image-20250108091953346](assets/image-20250108091953346.png)

可以看到默认单回波、直角坐标系、不开启imu、

可以使用 `ros2 topic hz /topic_name` 查看话题发布的频率

## 内外参

最全的方案，[SLAM各传感器的标定总结：Camera/IMU/LiDAR](https://blog.csdn.net/zardforever123/article/details/130030767)

### **外参**

![image-20250108100632321](assets/image-20250108100632321.png)

[外参标定](https://blog.csdn.net/m0_61812914/article/details/129771156)中最后结果指出，标定结果与手册中出厂设置差不多，就是只有平移部分。但是其中使用ros1，且需要移动雷达和imu，才能标定。github链接**[LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)**，

[ROS1安装链接](https://gitcode.csdn.net/65ed73e41a836825ed79999c.html?dp_token=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpZCI6NDU5NTgyMCwiZXhwIjoxNzM2ODE5NjU3LCJpYXQiOjE3MzYyMTQ4NTcsInVzZXJuYW1lIjoicXFfNDg0MzIwNTUifQ.K45Z8PDjCdxchU33i0ZgD18Y3eKFkErFqPjrUJIyB0s&spm=1001.2101.3001.6650.4&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7Ebaidujs_baidulandingword%7Eactivity-4-135859750-blog-143644827.235%5Ev43%5Epc_blog_bottom_relevance_base2&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7Ebaidujs_baidulandingword%7Eactivity-4-135859750-blog-143644827.235%5Ev43%5Epc_blog_bottom_relevance_base2)，安装完之后记得注释掉ros2相关的环境变量，不然终端会提示一些东西

```
#ros,使用时选择其中一个
source /opt/ros/noetic/setup.bash
#source /opt/ros/humble/setup.bash
#export LD_LIBRARY_PATH=/home/wenming/ws_livox/install/livox_interfaces/lib:$LD_LIBRARY_PATH
#source ~/buildmap/install/setup.bash
#source ~/bumblebee/install/setup.bash
```

### **内参**

[BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)其中写了一大堆参数，英文的看不太清楚。

但是看很多文章，IMU内参标定，标的就是**陀螺仪（Gyr）参数**和**速度计（Acc）参数**

- 陀螺仪参数描述了角速度测量的噪声特性，分为 **角速度随机噪声（gyr_n）** 和 **角速度随机游走噪声（gyr_w）**。
- 加速度计参数描述了加速度测量的噪声特性，分为 **加速度随机噪声（acc_n）** 和 **加速度随机游走噪声（acc_w）**。

IMU测量方程：[惯性导航 | 测量方程中的噪声模型与离散时间噪声模型](https://blog.csdn.net/qq_32761549/article/details/136341088)，就是取很多数据进行建模

![image-20250108110520749](assets/image-20250108110520749.png)

![image-20250108110847459](assets/image-20250108110847459.png)

链接如下：

- [IMU内参标定](https://blog.csdn.net/weixin_42681311/article/details/126109617)
- [对Livox_avia内置IMU-BMI088进行内参标定](https://blog.csdn.net/m0_61812914/article/details/129158194)，这个可以试一下

## 内参标定试验

[对Livox_avia内置IMU-BMI088进行内参标定](https://blog.csdn.net/m0_61812914/article/details/129158194),使用的工具[imu_utils](https://github.com/gaowenliang/imu_utils)

### 环境配置

- ROS1
- [imu_utils](https://github.com/gaowenliang/imu_utils)