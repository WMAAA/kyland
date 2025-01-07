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