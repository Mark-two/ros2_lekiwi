#include <rclcpp/rclcpp.hpp>                        // ROS2 C++ 客户端核心库，提供节点、话题、服务等功能
#include <sensor_msgs/msg/image.hpp>                  // ROS2 标准图像消息类型定义
#include <sensor_msgs/msg/camera_info.hpp>             // ROS2 相机内参消息类型，包含焦距和光心
#include <cv_bridge/cv_bridge.h>                      // ROS2 图像消息与 OpenCV Mat 之间的桥梁转换库
#include <geometry_msgs/msg/point_stamped.hpp>         // ROS2 带时间戳的3D坐标点消息
#include <image_transport/image_transport.hpp>         // ROS2 图像传输插件，支持压缩传输等

// OpenCV 与 CUDA 核心头文件
#include <opencv2/opencv.hpp>                         // OpenCV 主头文件，包含图像处理全部功能
#include <cuda_runtime_api.h>                         // CUDA 运行时 API，用于 GPU 显存分配和数据拷贝
#include <NvInfer.h>                                  // TensorRT 推理引擎核心头文件

#include <fstream>                                    // C++ 文件流，用于读取 .engine 二进制文件
#include <iostream>                                   // C++ 标准输入输出流
#include <vector>                                     // C++ 动态数组容器，用于存放模型数据和推理结果

using namespace nvinfer1;                             // 使用 TensorRT 命名空间，避免每次写 nvinfer1:: 前缀

// 1. 必须实现一个 TRT 的日志接收器 (TRT 规矩)
class Logger : public ILogger {                       // 继承 TensorRT 的 ILogger 接口，用于接收 TRT 内部日志
    void log(Severity severity, const char* msg) noexcept override {  // 重写日志回调函数，noexcept 保证不抛异常
        if (severity <= Severity::kWARNING) {         // 只打印 WARNING 及以上级别的日志（过滤掉 INFO/VERBOSE）
            std::cout << "[TRT] " << msg << std::endl; // 输出带 [TRT] 前缀的日志到控制台
        }                                             // if 结束
    }                                                 // log 函数结束
} gLogger;                                            // 全局日志实例，供 TensorRT 运行时使用

class YoloTrtNode : public rclcpp::Node {             // 定义 YOLO TensorRT 节点类，继承 ROS2 节点基类
public:                                               // 公有成员区域
    YoloTrtNode() : Node("yolo_trt_node") {           // 构造函数，注册节点名为 "yolo_trt_node"
        // --- 核心动作 1：声明话题 ---
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(  // 创建图像话题订阅器
            "/camera/camera/color/image_raw", 10,            // 订阅 RealSense 相机原始彩色图像话题，队列深度为10
            std::bind(&YoloTrtNode::image_callback, this, std::placeholders::_1));  // 绑定回调函数，收到图像时自动调用

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/lekiwi_vision/yolo_image", 10);  // 创建发布器，发布带检测框的图像到指定话题
        point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/lekiwi_vision/target_point", 10);  // 创建3D坐标发布器

        // --- 核心动作 2：极其优雅地读取外部 YAML 参数 ---
        // 1. 必须先"声明"参数，并给个兜底的默认值，防止没传 yaml 时程序直接崩溃
        this->declare_parameter<std::string>("engine_path", "default.engine");  // 声明字符串参数 engine_path，默认值为 "default.engine"
        
        // 2. 从 ROS 2 参数服务器里把值"抠"出来
        std::string engine_path;                      // 定义局部变量，用于接收参数值
        this->get_parameter("engine_path", engine_path);  // 从参数服务器获取 engine_path 的实际值

        RCLCPP_INFO(this->get_logger(), "✅ 成功读取配置文件，引擎路径: %s", engine_path.c_str());  // 打印 INFO 日志，确认引擎路径已加载

        // 3. 点火！
        init_tensorrt_engine(engine_path);            // 调用初始化函数，加载 TensorRT 引擎并分配 GPU 显存

        RCLCPP_INFO(this->get_logger(), "🚀 YOLO11 TensorRT 引擎挂载完毕，等待视频流...");  // 打印启动完成日志

        // 订阅相机内参，获取焦距和光心用于 3D 坐标计算
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                fx_ = msg->k[0];  // 焦距 fx
                fy_ = msg->k[4];  // 焦距 fy
                cx_ = msg->k[2];  // 光心 cx
                cy_ = msg->k[5];  // 光心 cy
                has_intrinsics_ = true;
            });

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/aligned_depth_to_color/image_raw", 10,  // 👈 注意这里是双 camera！
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                // 把 ROS 的 16 位深度图转成 OpenCV 矩阵 (单位: 毫米)
                latest_depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "深度图转换失败: %s", e.what());
            }
        });


    }                                                 // 构造函数结束

    ~YoloTrtNode() {                                  // 析构函数，节点销毁时自动调用
        // 熄火：释放 GPU 显存，防止内存泄漏
        cudaFree(buffers_[0]);                        // 释放输入缓冲区的 GPU 显存
        cudaFree(buffers_[1]);                        // 释放输出缓冲区的 GPU 显存
    }                                                 // 析构函数结束

private:                                              // 私有成员区域
    // ROS 2 核心通信组件
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  // 图像订阅器智能指针，管理订阅生命周期
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;        // 图像发布器智能指针，管理发布生命周期
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;  // 3D坐标发布器

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Mat latest_depth_image_;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;       // 相机内参：焦距和光心
    bool has_intrinsics_ = false;                      // 是否已收到内参

    // TensorRT 核心组件
    std::unique_ptr<IRuntime> runtime_;                // TensorRT 运行时实例，负责反序列化引擎
    std::unique_ptr<ICudaEngine> engine_;              // TensorRT 引擎实例，包含优化后的网络结构
    std::unique_ptr<IExecutionContext> context_;       // TensorRT 执行上下文，负责实际推理计算
    void* buffers_[2];                                // GPU 显存指针数组：[0]是输入图片，[1]是输出结果
    cudaStream_t stream_;                             // CUDA 异步流，用于并发执行 GPU 操作

    const int INPUT_W = 640;                          // 模型输入图像宽度（像素），YOLO 标准尺寸
    const int INPUT_H = 640;                          // 模型输入图像高度（像素），YOLO 标准尺寸
    const int OUTPUT_SIZE = 8400 * 5;                 // 模型输出总大小：8400个候选框 × (4个坐标 + 1个置信度)

    // ==========================================
    // 第一斧：读取 .engine 文件，在 GPU 上挖坑
    // ==========================================
    void init_tensorrt_engine(const std::string& engine_path) {  // TensorRT 引擎初始化函数，参数为引擎文件路径
        std::ifstream file(engine_path, std::ios::binary);  // 以二进制模式打开 .engine 文件
        if (!file.good()) {                           // 检查文件是否成功打开
            RCLCPP_ERROR(this->get_logger(), "找不到 Engine 文件！代驾小哥迷路了！");  // 打印错误日志
            return;                                   // 文件不存在则直接返回，不继续初始化
        }                                             // if 结束

        // 把文件读到物理内存
        file.seekg(0, file.end);                      // 将文件指针移到末尾，准备获取文件大小
        size_t size = file.tellg();                   // 获取文件总字节数
        file.seekg(0, file.beg);                      // 将文件指针移回开头，准备读取
        std::vector<char> trtModelStream(size);       // 开辟与文件大小相同的内存缓冲区
        file.read(trtModelStream.data(), size);       // 将整个 .engine 文件读入内存
        file.close();                                 // 关闭文件句柄，释放系统资源

        // 反序列化为 TRT 引擎
        runtime_.reset(createInferRuntime(gLogger));  // 创建 TensorRT 运行时实例，传入日志器
        engine_.reset(runtime_->deserializeCudaEngine(trtModelStream.data(), size));  // 反序列化二进制数据为 CUDA 引擎
        context_.reset(engine_->createExecutionContext());  // 从引擎创建执行上下文，用于后续推理

        // 在 GPU 上"挖"两块显存池（输入和输出）
        cudaMalloc(&buffers_[0], 1 * 3 * INPUT_H * INPUT_W * sizeof(float));  // 分配输入显存：1张图 × 3通道 × 640×640 × 4字节
        cudaMalloc(&buffers_[1], 1 * OUTPUT_SIZE * sizeof(float));            // 分配输出显存：8400×5 × 4字节
        cudaStreamCreate(&stream_);                   // 创建 CUDA 异步流，用于非阻塞的 GPU 操作
    }                                                 // init_tensorrt_engine 函数结束

    // ==========================================
    // 第二斧：视频流回调与极速推理
    // ==========================================
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {  // 图像回调函数，每收到一帧图像自动触发
        // 1. 将 ROS 图像无缝转换为 OpenCV 矩阵
        cv_bridge::CvImagePtr cv_ptr;                 // 定义 cv_bridge 智能指针，用于存转换后的图像
        try {                                         // 异常捕获块开始
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // 将 ROS Image 消息深拷贝为 OpenCV BGR8 格式
        } catch (cv_bridge::Exception& e) {           // 捕获 cv_bridge 转换异常
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 翻车: %s", e.what());  // 打印转换错误详情
            return;                                   // 转换失败则跳过本帧
        }                                             // try-catch 结束

        cv::Mat img = cv_ptr->image;                  // 获取原始图像的 OpenCV Mat 引用
        cv::Mat resized_img;                          // 定义缩放后的图像变量

        // 2. 图像预处理 (缩放到 640x640，HWC 转 CHW，归一化)
        cv::resize(img, resized_img, cv::Size(INPUT_W, INPUT_H));  // 将原图缩放到 640×640 模型输入尺寸
        std::vector<float> input_data(1 * 3 * INPUT_H * INPUT_W);  // 分配浮点数组：1×3×640×640，用于存预处理后的像素数据
        // 像素搬运：BGR→RGB + HWC→CHW + 归一化到 [0,1]
        for (int c = 0; c < 3; c++) {                 // 遍历 R/G/B 三个通道
            for (int h = 0; h < INPUT_H; h++) {       // 遍历每一行
                for (int w = 0; w < INPUT_W; w++) {   // 遍历每一列
                    // OpenCV 是 BGR，YOLO 要 RGB，所以通道映射：模型c=0→BGR[2]=R, c=1→BGR[1]=G, c=2→BGR[0]=B
                    input_data[c * INPUT_H * INPUT_W + h * INPUT_W + w] =
                        resized_img.at<cv::Vec3b>(h, w)[2 - c] / 255.0f;
                }
            }
        }

        // 3. 把 CPU 数据拷贝到 GPU 显存
        cudaMemcpyAsync(buffers_[0], input_data.data(), input_data.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);  // 异步将预处理数据从主机内存拷贝到 GPU 输入缓冲区

        // 4. ⚡️ 极速推理 (调用 Tensor Core) ⚡️
        // TRT 10 最新语法：绑定显存地址并执行
        context_->setTensorAddress("images", buffers_[0]);   // 将输入张量 "images" 绑定到 GPU 输入缓冲区地址
        context_->setTensorAddress("output0", buffers_[1]);  // 将输出张量 "output0" 绑定到 GPU 输出缓冲区地址
        context_->enqueueV3(stream_);                 // 在 CUDA 流上异步执行推理，利用 Tensor Core 加速计算

        // 5. 把结果从 GPU 拷贝回 CPU 内存
        std::vector<float> output_data(OUTPUT_SIZE);  // 在 CPU 端分配输出数组，大小为 8400×5
        cudaMemcpyAsync(output_data.data(), buffers_[1], output_data.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);  // 异步将推理结果从 GPU 拷贝回主机内存
        cudaStreamSynchronize(stream_);               // 阻塞等待 CUDA 流中所有操作完成，确保数据拷贝结束

        // ==========================================
        // 第三斧：后处理 — 解析 YOLO11 输出 + NMS
        // ==========================================
        const float CONF_THRESHOLD = 0.1f;            // 置信度阈值，低于此值的检测框直接丢弃
        const float NMS_THRESHOLD  = 0.45f;           // NMS IoU 阈值，重叠超过此比例的框只保留最优
        const int NUM_DETECTIONS   = 8400;            // YOLO11 固定输出 8400 个候选框

        // 计算从模型输入尺寸 (640x640) 映射回原始图像的缩放因子
        float scale_x = (float)img.cols / INPUT_W;    // 水平缩放因子
        float scale_y = (float)img.rows / INPUT_H;    // 垂直缩放因子

        std::vector<cv::Rect> boxes;                  // 存放所有候选检测框
        std::vector<float> confidences;               // 存放对应的置信度

        // YOLO11 输出格式 [5, 8400]：按行依次是 cx, cy, w, h, conf
        for (int i = 0; i < NUM_DETECTIONS; i++) {    // 遍历 8400 个候选框
            float conf = output_data[4 * NUM_DETECTIONS + i];  // 第 4 行是置信度
            if (conf < CONF_THRESHOLD) continue;      // 低置信度直接跳过

            float cx = output_data[0 * NUM_DETECTIONS + i] * scale_x;  // 中心 x 坐标映射到原图
            float cy = output_data[1 * NUM_DETECTIONS + i] * scale_y;  // 中心 y 坐标映射到原图
            float w  = output_data[2 * NUM_DETECTIONS + i] * scale_x;  // 宽度映射到原图
            float h  = output_data[3 * NUM_DETECTIONS + i] * scale_y;  // 高度映射到原图

            int x = static_cast<int>(cx - w / 2);     // 左上角 x = 中心x - 半宽
            int y = static_cast<int>(cy - h / 2);     // 左上角 y = 中心y - 半高

            boxes.emplace_back(x, y, static_cast<int>(w), static_cast<int>(h));  // 存入候选框
            confidences.push_back(conf);              // 存入对应置信度
        }

        // NMS 非极大值抑制：去除高度重叠的冗余框，只保留最优的
        std::vector<int> indices;                     // 存放 NMS 后保留的框索引
        cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, indices);

        // 在原图上绘制所有经过 NMS 筛选的检测框，并查询深度
        for (int idx : indices) {
            cv::Rect box = boxes[idx];                // 取出当前检测框
            cv::rectangle(img, box, cv::Scalar(0, 0, 255), 3);  // 画红色矩形框，线宽3
            std::string label = "Ball " + std::to_string(static_cast<int>(confidences[idx] * 100)) + "%";  // 拼接标签文字

            // 计算检测框中心点坐标，用于深度查询
            int center_x = box.x + box.width / 2;
            int center_y = box.y + box.height / 2;

            // 如果深度图可用，查询目标距离并追加到标签上
            if (!latest_depth_image_.empty() && 
                center_x >= 0 && center_x < latest_depth_image_.cols && 
                center_y >= 0 && center_y < latest_depth_image_.rows) {
                
                uint16_t depth_mm = latest_depth_image_.at<uint16_t>(center_y, center_x);
                
                if (depth_mm > 0) {
                    double z = depth_mm / 1000.0;     // 毫米转米
                    if (has_intrinsics_) {
                        // 针孔模型：像素坐标 + 深度 → 相机坐标系 3D 坐标
                        double x3d = (center_x - cx_) * z / fx_;
                        double y3d = (center_y - cy_) * z / fy_;
                        char coord_buf[64];
                        snprintf(coord_buf, sizeof(coord_buf), " (%.2f,%.2f,%.2f)m", x3d, y3d, z);
                        label += coord_buf;
                        RCLCPP_INFO(this->get_logger(), "🎯 锁定毛球！3D坐标: (%.3f, %.3f, %.3f) m", x3d, y3d, z);

                        geometry_msgs::msg::PointStamped point_msg;
                        point_msg.header = msg->header;
                        point_msg.point.x = x3d;
                        point_msg.point.y = y3d;
                        point_msg.point.z = z;
                        point_publisher_->publish(point_msg);
                    } else {
                        label += " " + std::to_string(depth_mm) + "mm";
                        RCLCPP_INFO(this->get_logger(), "🎯 锁定毛球！像素坐标: (%d, %d), 距离: %d mm", 
                                    center_x, center_y, depth_mm);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "⚠️ 找到毛球，但该点深度值为 0，测距失败！");
                }
            }

            cv::putText(img, label, cv::Point(box.x, box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);  // 在框上方画标签
        }

        // 把画好框的 OpenCV 图像转回 ROS 消息并发布出去！
        sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
        publisher_->publish(*out_msg);

    }                                                 // image_callback 函数结束
};                                                    // YoloTrtNode 类定义结束

int main(int argc, char * argv[]) {                   // 程序入口函数
    // ROS 2 节点初始化
    rclcpp::init(argc, argv);                         // 初始化 ROS2 通信框架，解析命令行参数
    rclcpp::spin(std::make_shared<YoloTrtNode>());    // 创建 YoloTrtNode 节点实例并进入事件循环，阻塞等待回调
    rclcpp::shutdown();                               // 退出事件循环后，清理 ROS2 资源
    return 0;                                         // 程序正常退出
}                                                     // main 函数结束
