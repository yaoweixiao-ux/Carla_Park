1. 将源码git clone 下来

```
git clone https://github.com/yaoweixiao-ux/Carla_Park.git
```

升级 OpenCV 到最新版本（建议 4.7 及以上）

```
conda install -c conda-forge opencv

```

要正确的运行必须从anconda环境中出来。

crist@crist:~/Desktop/Carla_Park$ pkg-config --modversion opencv4
4.2.0

sudo apt update
sudo apt install -y build-essential cmake git pkg-config
libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev
libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev
libopenexr-dev libeigen3-dev libdc1394-22-dev libqt5opengl5-dev qtbase5-dev

mkdir -p ~/SoftWare && cd ~/SoftWare

# OpenCV 主仓库

wget -O opencv-4.5.4.zip https://github.com/opencv/opencv/archive/4.5.4.zip
unzip opencv-4.5.4.zip

# contrib 模块

wget -O opencv_contrib-4.5.4.zip https://github.com/opencv/opencv_contrib/archive/4.5.4.zip
unzip opencv_contrib-4.5.4.zip

cd ~/SoftWare/opencv-4.5.4
mkdir build && cd build

```

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/opencv-4.5.4 \
  -DOPENCV_EXTRA_MODULES_PATH=~/SoftWare/opencv_contrib-4.5.4/modules \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DWITH_QT=ON \
  -DWITH_OPENGL=ON \
  -DWITH_TBB=ON \
  -DOPENCV_GENERATE_PKGCONFIG=ON

```

make -j$(nproc)
sudo make install

gedit ~/.bashrc

#让 CMake 中的 find_package(OpenCV) 找到 4.5.4 的配置。
export OpenCV_DIR=/opt/opencv-4.5.4

#让 CMake 优先在 /opt 查找 OpenCV 的 include 和 lib。
export CMAKE_PREFIX_PATH=/opt/opencv-4.5.4:$CMAKE_PREFIX_PATH

#让 pkg-config 正确输出 4.5.4 的版本和编译参数。
export PKG_CONFIG_PATH=/opt/opencv-4.5.4/lib/pkgconfig:$PKG_CONFIG_PATH

#运行 C++ 可执行程序时能正确加载 4.5.4 的 .so 动态库。
export LD_LIBRARY_PATH=/opt/opencv-4.5.4/lib:$LD_LIBRARY_PATH

catkin_make -DCMAKE_PREFIX_PATH=/opt/opencv-4.5.4

pip3 install ultralytics


运行：

rosrun yolo_obb_node CarlaSlotDetect.py
