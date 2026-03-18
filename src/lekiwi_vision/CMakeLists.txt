# =====================================================================
# CMakeLists.txt — lekiwi_vision 包的构建脚本
# 功能：在 Jetson 上用 TensorRT + CUDA 加速 YOLO 推理，并通过 ROS 2 发布结果
# =====================================================================

# 指定 CMake 最低版本要求为 3.8（ROS 2 Humble 推荐最低版本）
cmake_minimum_required(VERSION 3.8)

# 声明项目名称为 lekiwi_vision；后续 ${PROJECT_NAME} 都等于 "lekiwi_vision"
project(lekiwi_vision)

# 如果编译器是 GCC 或 Clang，则开启以下编译选项：
#   -Wall      : 开启所有常见警告
#   -Wextra    : 开启额外警告（比 -Wall 更严格）
#   -Wpedantic : 严格遵循 ISO C++ 标准，非标准写法会报警
#   -O3        : 最高级别优化（提升运行速度，适合部署）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ==========================================
# 1. 寻找 ROS 2 核心依赖
# ==========================================

# find_package() 会在系统路径中查找指定的包，REQUIRED 表示找不到就报错终止
# ament_cmake  : ROS 2 的 CMake 构建系统宏/函数集（必须第一个找）
find_package(ament_cmake REQUIRED)
# rclcpp       : ROS 2 C++ 客户端库（创建 Node、Publisher、Subscriber 等）
find_package(rclcpp REQUIRED)
# sensor_msgs  : ROS 2 传感器消息类型（Image、PointCloud2 等）
find_package(sensor_msgs REQUIRED)
# cv_bridge    : 在 ROS 的 sensor_msgs/Image 和 OpenCV 的 cv::Mat 之间做转换
find_package(cv_bridge REQUIRED)
# image_transport : 高效图像传输（支持 compressed / raw 等多种传输插件）
find_package(image_transport REQUIRED)
# std_msgs     : ROS 2 基础消息类型（String、Int32、Float64 等）
find_package(std_msgs REQUIRED)
# geometry_msgs: ROS 2 几何消息类型（Point、Pose、Twist 等）
find_package(geometry_msgs REQUIRED)

# ==========================================
# 2. 寻找外部视觉与计算库 (OpenCV, CUDA, TensorRT)
# ==========================================

# 查找 OpenCV 库；找到后自动设置 ${OpenCV_INCLUDE_DIRS} 和 ${OpenCV_LIBS}
find_package(OpenCV REQUIRED)

# set() 定义 CMake 变量
# TRT_LIBRARIES : TensorRT 的库名列表
#   nvinfer       — TensorRT 推理引擎核心库
#   nvonnxparser  — 将 ONNX 模型解析为 TensorRT 网络的库
set(TRT_LIBRARIES nvinfer nvonnxparser)

# CUDA_LIBRARIES : CUDA 运行时库
#   cudart — CUDA Runtime API（cudaMalloc、cudaMemcpy 等）
set(CUDA_LIBRARIES cudart)

# ==========================================
# 3. 编译 C++ 推理节点
# ==========================================

# add_executable() 声明一个可执行目标：
#   第 1 个参数 = 生成的可执行文件名（yolo_trt_node）
#   后续参数   = 需要编译的源文件列表
add_executable(yolo_trt_node src/yolo_trt_node.cpp)

# target_include_directories() 为目标添加头文件搜索路径
#   PUBLIC : 不仅本目标用，依赖本目标的其他目标也能用
#   $<BUILD_INTERFACE:...>  : 在 *编译时* 使用本地源码路径下的 include/ 目录
#   $<INSTALL_INTERFACE:...>: 在 *安装后* 使用安装目录下的 include/ 目录
#   ${OpenCV_INCLUDE_DIRS}  : OpenCV 头文件路径（由 find_package 自动填充）
#   /usr/local/cuda/include : CUDA 头文件路径（cuda_runtime.h 等）
#   /usr/include/aarch64-linux-gnu : Jetson (ARM64) 上 TensorRT 头文件的默认路径
target_include_directories(yolo_trt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/cuda/include
  /usr/include/aarch64-linux-gnu
)

# ament_target_dependencies() 是 ROS 2 提供的便利宏：
#   自动帮你把指定包的 include 路径、链接库、编译定义全部加到目标上
#   等价于手动调用 target_include_directories + target_link_libraries + ...
ament_target_dependencies(yolo_trt_node
  rclcpp            # ROS 2 C++ 节点框架
  sensor_msgs       # Image 消息
  cv_bridge         # ROS Image ↔ cv::Mat 转换
  image_transport   # 高效图像传输
  std_msgs          # 基础消息
  geometry_msgs     # 几何消息（如检测框坐标）
)

# target_link_directories() 为指定目标添加库文件搜索路径
#   /usr/local/cuda/lib64 : CUDA 运行时库（libcudart.so）所在目录
target_link_directories(yolo_trt_node PUBLIC /usr/local/cuda/lib64)

# target_link_libraries() 将外部库链接到可执行文件
#   ${OpenCV_LIBS}     : OpenCV 的所有库（core、imgproc、highgui 等）
#   ${TRT_LIBRARIES}   : TensorRT 库（nvinfer、nvonnxparser）
#   ${CUDA_LIBRARIES}  : CUDA 运行时库（cudart）
target_link_libraries(yolo_trt_node
  ${OpenCV_LIBS}
  ${TRT_LIBRARIES}
  ${CUDA_LIBRARIES}
)

# ==========================================
# 4. 安装规则 (供 colcon build 拷贝到 install/ 目录)
# ==========================================

# install(TARGETS ...) 安装编译好的可执行文件
#   DESTINATION lib/${PROJECT_NAME} : 安装到 install/lekiwi_vision/lib/lekiwi_vision/
#   这样 ros2 run lekiwi_vision yolo_trt_node 才能找到它
install(TARGETS
  yolo_trt_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/ ...) 安装头文件目录
#   将本包 include/ 下的所有头文件拷贝到 install/lekiwi_vision/include/
#   供其他 ROS 2 包依赖时使用
install(DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY launch config weights ...) 安装资源目录
#   launch/  : ROS 2 启动文件（.launch.py）
#   config/  : 参数配置文件（.yaml）
#   weights/ : 模型权重文件（.onnx / .engine 等）
#   urdf/    : 机器人描述文件（.urdf / .xacro 等）
#   meshes/  : 机械臂3D网格模型文件（.stl）
#   安装到 install/lekiwi_vision/share/lekiwi_vision/ 下
install(DIRECTORY launch config weights urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# ament_package() 是整个 CMakeLists.txt 的收尾宏（必须在最后调用）
#   它会生成 ament 索引文件，让 ROS 2 能够发现这个包
#   同时导出所有依赖信息，供 colcon 和其他包使用
ament_package()