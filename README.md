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

> 今天上午先查了一些livox avia相关资料，自带的imu资料，内外参。
>
> 然后下午标定了一下imu内参，但是相比于配置文件要小很多，问了gpt说太小就更信任imu，大一点鲁棒性更好
>
> 看了半篇综述，晚上回去做一下思维导图，搜了一篇关于占据栅格网络的文献，明天看一下

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
  - 还是用我的ubuntu20.04好用

### 录制rosbag

```
rosbag record -o imu_data.bag /livox/imu
```

### 标定内参

1.编写launch文件，在 imu_utils 下的 launch 文件夹中创建 imu_test.launch，其实复制一个launch改个名字，把里面需要填的数据填一下即可，不用自己动手一个

个输入：

```
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/livox/imu"/>
        <param name="imu_name" type="string" value= "avia_imu"/>
        <param name="data_save_path" type="string" value= "$(find imu_utils)/"/>
        <param name="max_time_min" type="int" value= "120"/>
        <param name="max_cluster" type="int" value= "100"/>
    </node>
</launch>
```

2.运行 imu_test.launch 文件

```
roslaunch imu_utils avia.launch
```

3.播放录制的包

```
rosbag play -r 100 ./data/imu_data.bag
```

等包播放完，终端会显示

![image-20250108163719774](assets/image-20250108163719774.png)

![image-20250108163732161](assets/image-20250108163732161.png)

现在程序开始标定了，结束后到保存的位置，找到相对应自己imu名字的yaml文件，标定数据在里面，如下：

![image-20250108164117240](assets/image-20250108164117240.png)

![image-20250108164202096](assets/image-20250108164202096.png)

# 2025/01/09

> 今天有点摆烂，上午装了一上午wifi驱动
>
> 下午修改雷达驱动，雷达的imu消息有问题，最后修改完成后，我试的avia雷达消息正常，但是塔吊现场雷达得lidar消息有点慢，只有6hz左右。
>
> 最后稍微读了一下雷达驱动流程，还没有解决问题。

## 网卡驱动

tenda ax300，直接去官网下载驱动，上午的驱动进bios，将安全模式关闭就可以安装了，还好没有瞎搞弄内核版本。。。新的wifi就是快，之前还以为是公司网络的问题呢。

## 修改雷达驱动

imu频率影响slam精度和鲁棒性，让我找一下为啥`livox_ros2_driver`跑出来的imu频率有问题，问题如下：

imu用topic hz查，是50hz，录bag包，显示是10hz

![image-20250109153747622](assets/image-20250109153747622.png)

### 版本一

加入线程相关函数，修改文件，**失败**，cpu占用率100

![image-20250109154018866](assets/image-20250109154018866.png)

### 版本二

版本也不对，虽然看着正常，但是随着时间越长，频率越来越低，误差越来越大，长时间运行不行

![image-20250109161443068](assets/image-20250109161443068.png)

### 版本三

手动进行imu间隔时间的设置，直接每次读取当前时间，等到指定的间隔时间

代码如下

```
void Lddc::PollingLidarImuData(uint8_t handle, LidarDevice *lidar) {
  // 获取激光雷达设备的IMU数据队列
  LidarDataQueue *p_queue = &lidar->imu_data;
  
  // 如果IMU数据队列为空，则直接返回
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  // 设置目标轮询间隔为5ms，意味着希望以200Hz的频率处理IMU数据
  const auto target_interval = std::chrono::microseconds(5000); // 200Hz = 5ms
  
  // 获取当前时间作为轮询开始时间
  auto next_time = std::chrono::steady_clock::now();

  // 如果IMU线程正在运行，进入循环进行数据处理
  while (imu_thread_running_[handle]) {
    // 如果IMU数据队列不为空，发布IMU数据
    if (!QueueIsEmpty(p_queue)) {
      PublishImuData(p_queue, 1, handle);
    }

    // 控制轮询间隔，确保下一次处理的时间间隔为5ms
    next_time += target_interval;
    
    // 使当前线程休眠，直到指定的时间点，精确控制轮询频率
    std::this_thread::sleep_until(next_time);
  }
}

void Lddc::DistributeLidarData(void) {
  // 检查Lidar数据集合是否为空，如果为空则退出
  if (lds_ == nullptr) {
    return;
  }

  // 等待信号量，保证该操作的线程同步
  lds_->semaphore_.Wait();

  // 遍历所有的激光雷达设备
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    
    // 获取当前激光雷达设备的指针
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    
    // 获取该激光雷达设备的数据队列
    LidarDataQueue *p_queue = &lidar->data;
    
    // 如果设备未连接或数据队列为空，跳过该设备
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }

    // 调用函数处理该激光雷达的数据（点云数据）
    PollingLidarPointCloudData(lidar_id, lidar);

    // 启动或保持IMU数据的轮询线程
    std::lock_guard<std::mutex> lock(imu_thread_mutex_);  // 确保线程安全
    if (!imu_thread_running_[lidar_id]) {
      // 如果该激光雷达的IMU线程未启动，则启动新的线程
      imu_thread_running_[lidar_id] = true;

      // 创建IMU轮询线程
      std::thread imu_thread([this, lidar_id, lidar]() {
        // 在该线程中轮询IMU数据，直到IMU线程被停止
        while (imu_thread_running_[lidar_id]) {
          PollingLidarImuData(lidar_id, lidar);  // 调用PollImuData处理IMU数据
        }
      });

      // 分离线程，线程将在后台运行
      imu_thread.detach();
    }
  }

  // 如果接收到退出请求，则停止所有IMU线程并准备退出
  if (lds_->IsRequestExit()) {
    for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
      imu_thread_running_[i] = false; // 停止所有IMU线程
    }
    // 调用退出准备函数
    PrepareExit();
  }
}
```

修改后的效果

![image-20250109162811366](assets/image-20250109162811366.png)

## 读雷达驱动相关

### 主要流程

`livox_ros2_driver.cpp`文件最后，注册了一个 ROS 2 节点，允许它作为一个组件节点运行。

```
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::LivoxDriver)
```

去看构造函数

```
LivoxDriver::LivoxDriver(const rclcpp::NodeOptions & node_options)
```

`GetInstance`函数，设置了间隔时间，单位为ms

`InitLdsLidar`中说了，如果白名单中没有广播码的话，使用自动连接

其实很多代码都没有用，都是if，else，判断，else都没执行，因为是直接连接雷达。

然后就进入了多线程

```
  poll_thread_ = std::make_shared<std::thread>(&LivoxDriver::pollThread, this);
```

然后，用于在一个独立的线程中持续处理 LiDAR 数据的分发

```
void LivoxDriver::pollThread()
{
  std::future_status status;

  do {
    lddc_ptr_->DistributeLidarData();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}
```

具体来说就是调用`DistributeLidarData`函数，这个就是在`lddc`中，雷达和imu分开发的？，在这个函数中遍历所有的`lidar`，然后分别处理点云和imu数据，`PollingLidarPointCloudData`和`PollingLidarImuData`

因为`imu`的函数我已经修改过了，所以主要看处理雷达点云数据的函数

```
void Lddc::PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar) {
  // 获取 LiDAR 设备的存储数据队列
  LidarDataQueue *p_queue = &lidar->data;

  // 如果数据队列的存储空间为空，直接返回
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  // 循环直到队列为空或队列中数据不足以发布一次数据包
  while (!QueueIsEmpty(p_queue)) {
    uint32_t used_size = QueueUsedSize(p_queue);  // 获取当前队列中使用的数据量
    uint32_t onetime_publish_packets = lidar->onetime_publish_packets;  // 每次发布的数据包数量

    // 如果当前队列中的数据不足以发布一次数据包，则退出循环
    if (used_size < onetime_publish_packets) {
      break;
    }

    // 根据不同的传输格式发布数据
    if (kPointCloud2Msg == transfer_format_) {
      // 如果传输格式为 PointCloud2 消息类型，调用 PublishPointcloud2 函数发布数据
      PublishPointcloud2(p_queue, onetime_publish_packets, handle);
    } else if (kLivoxCustomMsg == transfer_format_) {
      // 如果传输格式为 Livox 自定义消息类型，调用 PublishCustomPointcloud 函数发布数据
      PublishCustomPointcloud(p_queue, onetime_publish_packets, handle);
    } else if (kPclPxyziMsg == transfer_format_) {
      // 如果传输格式为 PCL 的 PXZ/ZI 数据类型，调用 PublishPointcloudData 函数发布数据
      PublishPointcloudData(p_queue, onetime_publish_packets, handle);
    }
  }
}
```

因为我要用的是`PublishCustomPointcloud`，

看完以后觉得是这俩参数的问题

- `onetime_publish_packets`，每秒的点除以每个包的数量

```
    p_lidar->onetime_publish_packets = \
        GetPacketNumPerSec(p_lidar->info.type, \
        p_lidar->raw_data_type) * buffer_time_ms_ / 1000;
```

- `packet_interval_max`

  ```
    int64_t packet_gap = timestamp - last_timestamp;
  
    if ((packet_gap > lidar->packet_interval_max) &&
  
  ​    lidar->data_is_pubulished) {
  ```

  

### PublishCustomPointcloud

1. 如果无法获取起始时间，或者队列中的数据包数量不足，则返回 `0`

2. 创建并初始化消息结构体

   ```
     livox_interfaces::msg::CustomMsg livox_msg;
     livox_msg.header.frame_id.assign(frame_id_);
     livox_msg.timebase = 0;
     livox_msg.point_num = 0;
     livox_msg.lidar_id = handle;
   ```

3. 获取lidar参数并初始化数据缓冲区，这里我看到了`GetPointInterval`函数

   ```
     uint8_t point_buf[2048];
     uint8_t data_source = lds_->lidars_[handle].data_src;
     uint32_t line_num = GetLaserLineNumber(lidar->info.type);
     uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
     uint32_t point_interval = GetPointInterval(lidar->info.type);
     uint32_t published_packet = 0;
     uint32_t packet_offset_time = 0;  /** uint:ns */
     uint32_t is_zero_packet = 0;
   ```

4. 循环处理

   ```
   while (published_packet < packet_num) {
   ```

   ```
   从队列中获取数据包：
   QueuePrePop(queue, &storage_packet) 从队列中取出一个数据包。
   raw_packet 通过 reinterpret_cast 将原始数据转换为 LivoxEthPacket 指针。
   timestamp 获取当前数据包的时间戳。
   时间间隔检查：
   计算当前数据包与上一个数据包之间的时间差（packet_gap）。
   如果时间差超过最大允许时间间隔，并且数据已经发布过，则调整时间戳并将数据包置为空（ZeroPointDataOfStoragePacket），并设置 is_zero_packet 为 1，表示当前包是一个零数据包。
   ```

   思考一下影响雷达信息发布的频率的因素：

   - **`timestamp` 与 `last_timestamp`**：在 `PublishCustomPointcloud` 中，`timestamp` 和 `last_timestamp` 控制了每个数据包发布的时间间隔。如果相邻两个数据包的时间间隔超过了预设的最大时间间隔 `lidar->packet_interval_max`，则会调整时间戳并可能丢弃一些点数据，以控制数据发布的频率。
   - **时间间隔的计算**：`packet_gap` 是当前数据包与上一个数据包时间戳的差值，控制了发布的时间间隔。如果时间间隔过大，数据会被调整或丢弃（`ZeroPointDataOfStoragePacket`）。
   - **`packet_interval_max`**：这个参数是用于调整每个 LiDAR 数据包的最大间隔，用来避免发布的频率过高。

   - **`lidar->onetime_publish_packets`**：控制每次发布的数据包数量，间接影响了发布频率。每次发布的点云包数越多，发布的频率会相对降低。
   - **`lidar->packet_interval_max` 和 `lidar->packet_interval`**：这些参数限制了数据包之间的最大时间间隔和理想的时间间隔，它们直接影响到每秒钟发布的数据量

   

5. 处理第一个数据包为当前时间戳，后续的数据包设置为偏移量

   ```
       /** first packet */
       if (!published_packet) {
         livox_msg.timebase = timestamp;
         packet_offset_time = 0;
         /** convert to ros time stamp */
         livox_msg.header.stamp = rclcpp::Time(timestamp);
       } else {
         packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
       }
       uint32_t single_point_num = storage_packet.point_num * echo_num;
   ```

6. 点云数据转换与填充消息

7. 更新队列并准备下一包

8. 发布消息到 ROS



### GetPointInterval

内联函数 `GetPointInterval`，它根据 `product_type` 返回该产品类型对应的点间隔 (`point_interval`)

```
inline uint32_t GetPointInterval(uint32_t product_type) {
  return product_type_info_pair_table[product_type].point_interval;
}
```

`product_type_info_pair_table`

```
const ProductTypePointInfoPair product_type_info_pair_table[kMaxProductType] = {
    {100000, 10000, 1},
    {100000, 10000, 1},
    {240000, 4167 , 6}, /**< tele */
    {240000, 4167 , 6},
    {100000, 10000, 1},
    {100000, 10000, 1},
    {100000, 10000, 1}, /**< mid70 */
    {240000, 4167,  6},
    {240000, 4167,  6},
};
```

```
typedef struct {
  uint32_t points_per_second; /**< number of points per second 每秒钟多少个点 */
  uint32_t point_interval;    /**< unit:ns 相邻两点的时间间隔 */
  uint32_t line_num;          /**< laser line number */
} ProductTypePointInfoPair;
```

### FillPointsToCustomMsg

主要是这个时间戳的设置，

```
      point.offset_time = offset_time + i * point_interval;
```

数据包offset_time，其中的第i个点，累积多个数据包

```
void Lddc::FillPointsToCustomMsg(livox_interfaces::msg::CustomMsg& livox_msg, \
    LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time, \
    uint32_t point_interval, uint32_t echo_num) {
  LivoxPointXyzrtl* point_xyzrtl = (LivoxPointXyzrtl*)src_point;
  for (uint32_t i = 0; i < num; i++) {
    livox_interfaces::msg::CustomPoint point;
    if (echo_num > 1) { /** dual return mode */
      point.offset_time = offset_time + (i / echo_num) * point_interval;
    } else {
      point.offset_time = offset_time + i * point_interval;
    }
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.reflectivity = point_xyzrtl->reflectivity;
    point.tag = point_xyzrtl->tag;
    point.line = point_xyzrtl->line;
    ++point_xyzrtl;
    livox_msg.points.push_back(point);
  }
}
```

# 2025/01/10

> 今天整的事情大致如下：
>
> 1. 修改雷达驱动，根据[github思路](https://github.com/Livox-SDK/livox_ros2_driver/issues/39)修改`FillPointsToCustomMsg`函数
> 2. 跑一下`egoplanner`或`fastplanner`，看一下大致思路？
> 3. 速通那个占据网络概述，最好能跑一个开源框架的demo

## FillPointsToCustomMsg

简单的修改了一下，但是，塔吊现场的雷达，没有区别。

## 复现ego-planner

[源码地址](https://github.com/ZJU-FAST-Lab/ego-planner)

编译报错

```
CMake Error at /opt/ros/noetic/share/cv_bridge/cmake/cv_bridgeConfig.cmake:113 (message):
  Project 'cv_bridge' specifies '/usr/local/include/opencv' as an include
  dir, which is not found.  It does neither exist as an absolute directory
  nor in '${{prefix}}//usr/local/include/opencv'.  Check the issue tracker
  'https://github.com/ros-perception/vision_opencv/issues' and consider
  creating a ticket if the problem has not been reported yet.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:76 (find_package)
  planner/plan_env/CMakeLists.txt:12 (find_package)
```

由于我是ros1和ros2同时装，[问题解决](https://blog.csdn.net/Xjsdhwdma/article/details/135274908)

## 读综述

云里雾里的，都是专业名词，看的头疼，打算直接问chat

[Occ感知自动驾驶 Occupany network占用预测网络](https://blog.csdn.net/weixin_57950978/article/details/140355306)

[OccNet栅格占据网络](https://blog.csdn.net/Evelynnzhao/article/details/137427941)                                          

[Occ3D知乎论文介绍](https://zhuanlan.zhihu.com/p/634770039)                                            [Occ3D源码地址](https://tsinghua-mars-lab.github.io/Occ3D/)

## 复现

# 2025/01/13

> - 先看了一下首帧重力初始化怎么搞。结果源码已经搞过了
> - 想看一下buildmap代码，看如何加入的后端。ps：现在看了一部分
> - 目前还是雷达驱动有问题，达不到10hz

## 首帧重力对齐

参考链接[FASTLIO2_SAM_LC](https://github.com/liangheming/FASTLIO2_SAM_LC)

`IMU_init`函数中有重力初始化呀，就是累积求平均值，然后作为重力的方向。

```
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);
```

使用正则表达式，找S2函数调用![image-20250113113055439](assets/image-20250113113055439.png)

![image-20250113110830114](assets/image-20250113110830114.png)

[嘿嘿，好用](https://blog.csdn.net/Dontla/article/details/129289224)

## loop closure思路

发现了一个好东西，**[SLAM-application](https://github.com/engcang/SLAM-application)**集合了各种激光slam方案，

## 源码和修改版区别

| `updatePath`                | 更新里程计轨迹，四元数轨迹位姿                               |
| --------------------------- | ------------------------------------------------------------ |
| `transformPointCloud`       | `T_w_lidar  =  T_w_b * T_b_lidar`雷达到世界坐标系，点云都变换过去 |
| `transformPointCloudBody`   | 点云变换，雷达到body                                         |
| 重载函数`pointDistance`     | 点到原点，两点之间的距离                                     |
| `allocateMemory`            | 初始化存储空间                                               |
| `EulerToQuat`               | 欧拉角转四元数                                               |
| `getCurPose`                | 当前帧的位姿，并赋值到`transformTobeMapped[6]`，RPY,XYZ      |
| `visualizeLoopClosure`      | visualization_msgs::msg::MarkerArray markerArray;，就发布这玩意 |
| `saveFrame`                 | 关键帧判断，和上一帧的相对变换，角度或平移距离太小就不作为关键帧 |
| `addOdomFactor`             | 添加激光里程计因子                                           |
| `addLoopFactor`             | 添加闭环因子                                                 |
| `saveKeyFramesAndFactor`    | 在图优化过程中处理关键帧的保存和因子的添加                   |
| `recontructIKdTree`         | 根据当前的关键帧数据重建并更新 Kd-tree                       |
| `correctPoses`              | 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹 |
| `detectLoopClosureDistance` |                                                              |
| `loopFindNearKeyframes`     | 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样  |
| `performLoopClosure`        |                                                              |
| `trans2gtsamPose`           | 转化为`gtsam::Pose3`格式                                     |
|                             |                                                              |
|                             |                                                              |

### addOdomFactor

判断`cloudKeyPoses3D->points.empty()`

- 对于第一帧，初始化先验因子。

- 前一阵`poseFrom`当前帧`poseTo`

> 这个噪声协方差是怎么确定的？

```
void addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // 第一帧初始化先验因子
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else
    {
        // 添加激光里程计因子
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); /// pre
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // cur
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}
```

### addLoopFactor

闭环因子是用于图优化的约束，表示机器人在某个时刻回到了之前访问过的位置。这种约束帮助优化过程修正机器人的轨迹，消除累计的误差。闭环检测的目的是检测到这种回环并在图中添加相应的约束。

```
void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    // 闭环队列
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first; //   cur
        int indexTo = loopIndexQueue[i].second;  //    pre
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}
```

### saveKeyFramesAndFactor

- `saveFrame()`判断两帧差别，太小的话不作为关键帧。

- 添加因子

  ```
      // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
      addOdomFactor();
      // GPS因子 (UTM -> WGS84)
      //addGPSFactor();
      //addTurnFactor();
      // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
      addLoopFactor();
  ```

- 图优化更新，

  ```
      isam->update(gtSAMgraph, initialEstimate);
      isam->update();
      if (aLoopIsClosed == true) // 有回环因子，多update几次
      {
          isam->update();
          isam->update();
          isam->update();
          isam->update();
          isam->update();
      }
  ```

  > 为什么有回环因子，要多更新几次？次数怎么确定的？

- 清空因子图

- 优化结果和当前帧位姿

  ```
      // 优化结果
      isamCurrentEstimate = isam->calculateBestEstimate();
      // 当前帧位姿结果
      latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
  ```

- 将新关键帧放入队列，`cloudKeyPoses3D`和`cloudKeyPoses6D`

  > 为什么要分为3D和6D

- 更新状态量

- 当前帧激光角点、平面点，降采样集合

- `updatePath(thisPose6D);` //  可视化update后的path

### recontructIKdTree

1. **Kd-tree 初始化与输入数据准备**：

   - 创建一个 Kd-tree (`kdtreeGlobalMapPoses`) 用于存储全局关键帧位姿数据。

   - 创建多个点云对象：`subMapKeyPoses`、`subMapKeyPosesDS`、`subMapKeyFrames` 和 `subMapKeyFramesDS`，用于存储子地图的关键帧位姿和对应的特征点云数据。

2. **搜索附近关键帧**：

   - 使用 `kdtreeGlobalMapPoses->radiusSearch` 在全局地图中查找与当前关键帧相邻的关键帧。找到的相邻关键帧会被存储在 `subMapKeyPoses` 中。

3. **降采样**：

   - 使用 `pcl::VoxelGrid` 进行降采样。首先对 `subMapKeyPoses` 进行降采样，生成 `subMapKeyPosesDS`，
   - 然后根据降采样后的关键帧位姿 (`subMapKeyPosesDS`)，提取对应的局部地图特征点云 (`subMapKeyFrames`)，
   - 并对其进行降采样，最终生成 `subMapKeyFramesDS`，用于后续的 Kd-tree 重建。

4. **构建 Kd-tree**：
   - 使用subMapKeyFramesDS，`重新构建 Kd-tree`，调用 ikdtree.reconstruct(subMapKeyFramesDS->points) 来完成。


5. **更新与输出信息**：、
   - 输出 Kd-tree 的大小和从地图中提取到的特征点数量。


每次更新 `updateKdtreeCount`。加一

### correctPoses()

1. **检查是否有关键帧**：如果 `cloudKeyPoses3D` 中没有点云数据，直接返回。

2. **闭环检测**：如果闭环已被检测到 (`aLoopIsClosed == true`)，则执行以下操作：

   - **清空里程计轨迹**：`globalPath.poses.clear()` 清空当前的里程计轨迹。

   - **更新因子图中所有变量节点的位姿**：
     - 从 `isamCurrentEstimate` 中获取估计的位姿，并更新 `cloudKeyPoses3D` 和 `cloudKeyPoses6D` 中的关键帧位姿。
     - `cloudKeyPoses3D` 存储平移信息，`cloudKeyPoses6D` 存储平移和旋转（姿态）信息。

   - **更新里程计轨迹**：调用 `updatePath()` 更新轨迹。

   - **重建局部地图并重建 Kd 树**：调用 `recontructIKdTree()` 清空局部地图，并重建 Kd 树。

   - **重置闭环检测标志**：将 `aLoopIsClosed` 设置为 `false`，表示当前闭环已处理。

### detectLoopClosureDistance

1. **跳过已处理的关键帧**：
   - 如果当前关键帧（`loopKeyCur`）已经存在于 `loopIndexContainer` 中（表示已经添加过闭环信息），则直接返回 `false`，跳过闭环检测。
2. **查找历史关键帧中距离当前关键帧最近的帧**：
   - 使用 `kdtreeHistoryKeyPoses` 对历史关键帧构建一个 Kd 树，以加速查找。
   - 使用 `radiusSearch` 函数根据距离和半径 `historyKeyframeSearchRadius` 查找与当前关键帧最接近的关键帧。
3. **过滤掉时间差过小的候选帧**：
   - 在候选的历史关键帧中，使用时间差过滤条件（`historyKeyframeSearchTimeDiff`）来确保选择的候选帧与当前关键帧的时间相差较远，避免选择过于接近的帧。
4. **返回找到的回环关键帧的 ID**：
   - 如果找到符合条件的回环帧，返回当前关键帧和回环帧的 ID，供后续使用。
5. **返回 `false`**：
   - 如果没有找到合适的回环帧，或者当前帧和回环帧相同，则返回 `false`

### loopFindNearKeyframes

1. **清空输入点云**：首先清空 `nearKeyframes`，准备存储提取的关键帧特征点集合。
2. **获取关键帧集合大小**：
   - `cloudSize`：表示关键帧位姿的集合大小。
   - `surfcloud_keyframes_size`：表示特征点云集合 `surfCloudKeyFrames` 的大小。
3. **提取关键帧特征点集合**：
   - 根据给定的 `key` 和 `searchNum`，从当前关键帧向前和向后查找相邻的关键帧。总共查找 `2*searchNum + 1` 帧的关键帧特征点。
   - 使用 `transformPointCloud` 将每一帧的点云通过当前关键帧的位姿进行转换（从局部坐标系转换到全局坐标系）。
   - 默认使用 `surfCloudKeyFrames` 中的点云，因为 `fast-lio` 没有进行特征提取，直接使用 `surfCloud` 点云。
4. **检查是否有找到有效点云**：如果没有找到任何有效的关键帧点云（`nearKeyframes` 为空），则返回。
5. **降采样**：
   - 使用 `pcl::VoxelGrid` 进行降采样操作，减少点云中的点数，`downSizeFilterICP` 是降采样滤波器。
   - 降采样后的点云存储在 `cloud_temp` 中，并将结果赋值回 `nearKeyframes`。

### performLoopClosure

1. **检查是否有关键帧**：如果没有任何关键帧，函数直接返回。

2. **获取当前关键帧的历史数据**：使用 `mtx.lock()` 和 `mtx.unlock()` 保护共享资源，确保线程安全。将当前的 `cloudKeyPoses3D` 和 `cloudKeyPoses6D` 数据拷贝到 `copy_cloudKeyPoses3D` 和 `copy_cloudKeyPoses6D` 中。

3. **检测回环闭合**：通过 `detectLoopClosureDistance()` 函数找到与当前关键帧距离最小且时间相隔较远的历史关键帧作为候选回环帧。

4. **提取并降采样关键帧点云**：

   - 调用 `loopFindNearKeyframes()` 

   - 发布闭环匹配关键帧局部map

     > 这是怎么查询的？？？里面好像没看到特征点提取的过程呀

5. **ICP匹配**：通过ICP算法进行当前关键帧与历史关键帧的点云匹配。如果匹配收敛且匹配误差小于阈值，则继续进行后续处理。

   ```
       // ICP Settings
       pcl::IterativeClosestPoint<PointType, PointType> icp;
       icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
       icp.setMaximumIterations(100);
       icp.setTransformationEpsilon(1e-6);
       icp.setEuclideanFitnessEpsilon(1e-6);
       icp.setRANSACIterations(0);
   
       // scan-to-map，调用icp匹配
       icp.setInputSource(cureKeyframeCloud);
       icp.setInputTarget(prevKeyframeCloud);
       pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
       icp.align(*unused_result);
   
       // 未收敛，或者匹配不够好
       if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
           return;
   
       std::cout << "icp  success  " << std::endl;
   ```

6. **发布闭环优化后的点云**：如果匹配成功，则根据ICP计算的最终变换，发布闭环优化后的当前关键帧点云。

7. **计算闭环优化前后的位姿变化**：通过ICP优化的变换计算当前帧和闭环帧的相对位姿变化，并将结果保存到 `loopPoseQueue` 中。

8. **更新回环因子**：

   ```
       mtx.lock();
       loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
       loopPoseQueue.push_back(poseFrom.between(poseTo));
       loopNoiseQueue.push_back(constraintNoise);
       mtx.unlock();
   ```

9. **存储回环信息**：使用 `loopIndexContainer` 存储当前关键帧与闭环关键帧的索引对，确保不会重复进行回环检测。

> 这些东西会清除吗。
>
> ```
>     // 添加闭环因子需要的数据
>     mtx.lock();
>     loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
>     loopPoseQueue.push_back(poseFrom.between(poseTo));
>     loopNoiseQueue.push_back(constraintNoise);
>     mtx.unlock();
> 
>     loopIndexContainer[loopKeyCur] = loopKeyPre; //   使用hash map 存储回环对
> ```

### loopClosureThread

```
    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
        {
            std::cout << "loopClosureEnableFlag   ==  false " << endl;
            return;
        }

        std::cout << "loopClosureThread" << std::endl;
        performLoopClosure(pubHistoryKeyFrames, pubIcpKeyFrames);   //  回环检测
        visualizeLoopClosure(pubLoopConstraintEdge); // rviz展示闭环边
        
    }
```

### publish_path

### publish_path_update

### 调用闭环线程

就是设置了一个定时器，200ms执行一次。

```

        auto loopClosure_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 5.0));
        loopClosure_timer_ = rclcpp::create_timer(this, this->get_clock(), loopClosure_period_ms, std::bind(&LaserMappingNode::loopClosureThread, this));
        
        map_save_srv_ = this->create_service<buildmap::srv::SaveMap>("savemap", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));
```

> 有个问题，在哪里启动的线程呢？？

## 雷达驱动

- 思路一：[数据循环方面存在性能问题](https://github.com/Livox-SDK/Livox-SDK2/issues/22#issuecomment-1465588446)看`Driver2`和`ros2_driver`的区别。
  - 看了没啥区别呀

- 思路二：[其中的数据有一部分丢了](https://github.com/Livox-SDK/livox_ros_driver2/issues/56)
  - ros2_driver中没有找到类似的呀
- 思路三：[关于FillPointsToCustomMsg函数的思考](https://github.com/Livox-SDK/livox_ros2_driver/issues/39)

# 2025/01/14

> - 看动手学强化学习
> - 重读3D Occ论文综述

# 2025/01/15

> - 继续看，修改的回环检测部分的代码。继续总结到01/13的笔记中。
> - 等书到以后，复习动手学强化学习基础篇

安装一个`pycharm`

[保姆教程](https://blog.csdn.net/m0_74115845/article/details/142035764)

# 2025/01/16

> - 上午仍然是看雷达驱动，很奇怪，不知道为不同电脑上，lidar的频率不一样。跟厂家汇报了下
> - 复习动手学强化学习
> - 看了两篇论文
>   - `LTA-OM`：更偏重于多个session的对齐
>   - `LT-mappe`：这里有关于移除地图中不一样的地方。

# 2025/01/17

> - 复习一下，MPC，大致看一下陈虹的书。
> - 运行LT-SLAM
> - 使用SC回环检测
>   - 几乎是检查不到回环。

在 [Ubuntu 22.04 LTS](https://www.yundongfang.com/Yuntag/ubuntu-22-04-lts) Jammy Jellyfish 上[安装 Foxit PDF Reader](https://www.yundongfang.com/Yuntag/安装-foxit-pdf-reader)，，[教程](https://www.yundongfang.com/Yun5653.html)

## LT-SLAM教程

> 首先看这个！！！！！！！！！！！！！！！！！！！先创好架构
>
> ![image-20250117155258713](assets/image-20250117155258713.png)

### 数据准备

1. 准备数据

   ![image-20250117100737231](assets/image-20250117100737231.png)

2. [file_player_mulran](https://github.com/RPM-Robotics-Lab/file_player_mulran),下载这个,就是用来播放数据集，

   - ```
     source devel/setup.bash
     roslaunch file_player file_player.launch
     ```

   - 直接打开，`load`![image-20250117101556967](assets/image-20250117101556967.png)

3. [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)，

   - 修改一下文件保存的路径，`params_mulran.yaml`文件,路径设为

     - ![image-20250117104358727](assets/image-20250117104358727.png)

   - 启动建图

     ```
     roslaunch lio_sam run_mulran.launch 
     ```

     `file_player`播放即可，

     注：对于第一个数据集，先暂停一下，看看文件是否正确存储。

4. 对于01和02两个轨迹，起始位置不同，轨迹也没对齐，**LT-SLAM目标是对齐他们**

   ![image-20250117110015174](assets/image-20250117110015174.png)

### 合并轨迹

运行lLT-SLAM

- 新建一个`data_ltslam/0102`，修改序列数据库的路径![image-20250117110932514](assets/image-20250117110932514.png)

- 运行

  ```
  roslaunch ltslam run.launch
  ```

- 成功合并两条轨迹，使用半径搜索![image-20250117112042105](assets/image-20250117112042105.png)

**检查**

需要来两个文件，"_central_aft_intersession_loops.txt"，central意思是他们在共享坐标系下。同样可以在cloudcompare下打开，

点云地图检查,修改以下文件

```
/home/wenming/ltslam/catkin_scliosam/src/SC-LIO-SAM/tools/python/makeMergedMap.py
```

```
data_dir = "/home/wenming/ltslam/data_parsed/01/" # should end with / 
scan_idx_range_to_stack = [0, len(scan_files)] # if you want a whole map, use [0, len(scan_files)]

f = open(data_dir+"01_central_aft_intersession_loops.txt", 'r')
```

运行脚本文件

```
python3 makeMergedMap.py 
```

一直报错，[解决](https://blog.csdn.net/Tammy_lzh/article/details/133934959)

### 地图去除

修改配置文件

![image-20250117155111088](assets/image-20250117155111088.png)

运行

```
roslaunch removert run_ltmapper.launch 
```

报超出范围的错误，因为两个pcd的scans数量不同，用的多的那个作为循环的。

# 2025/01/20

> 1. 读SC论文。
> 2. 读Fast-lio-slam中位姿图优化的代码。
> 3. 看一下官网GTSAM

## Fast-lio-slam代码

scLoopICPBufscLoopICPBuf六个`thread`，

### process_pg

<img src="assets/image-20250120142106662.png" alt="image-20250120142106662" />

### process_lcd

![image-20250120142846303](assets/image-20250120142846303.png)

### performSCLoopClosure

![image-20250120143919071](assets/image-20250120143919071.png)

### process_icp

![image-20250120145356297](assets/image-20250120145356297.png)

### doICPVirtualRelative

![image-20250120152853901](assets/image-20250120152853901.png)

### process_isam

![image-20250120150022664](assets/image-20250120150022664.png)

## GTSAM相关

