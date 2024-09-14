#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <filesystem>
#include <fstream>

#define server_ip "192.168.1.103"   // 服务器的IP地址
#define server_port 10005           // 本树莓派与服务器通信的端口

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)


using namespace message_filters::sync_policies;
using namespace std::chrono_literals;

class CameraSyncNode : public rclcpp::Node
{
public:
    CameraSyncNode() : Node("camera_sync_node"), capture_flag_(false), image_count_(0), save_count_(20)
    {
        // 声明并初始化参数，包括ros2话题名称
        this->declare_parameter("camera1_topic", "camera1_10005_image");
        this->declare_parameter("camera2_topic", "camera2_10005_image");
        this->declare_parameter("camera3_topic", "camera3_10005_image");
        this->declare_parameter("stitched_image_topic", "stitched_10005_image");
        this->declare_parameter("save_directory", "/home/lamps-xe5/ros2_ws/image");
        this->declare_parameter("save_count", 20); // 添加 save_count 参数

        // 获取参数值
        camera1_topic_ = this->get_parameter("camera1_topic").as_string();
        camera2_topic_ = this->get_parameter("camera2_topic").as_string();
        camera3_topic_ = this->get_parameter("camera3_topic").as_string();
        stitched_image_topic_ = this->get_parameter("stitched_image_topic").as_string();
        save_directory_ = this->get_parameter("save_directory").as_string();
        save_count_ = this->get_parameter("save_count").as_int(); // 获取 save_count 参数

        // 订阅三个相机的图像话题
        sub1_.subscribe(this, camera1_topic_);
        sub2_.subscribe(this, camera2_topic_);
        sub3_.subscribe(this, camera3_topic_);

        // 创建用于发布拼接图像的发布器
        stitched_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(stitched_image_topic_, 10);

        // 设置消息同步器，并将同步后的回调函数注册
        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
            ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>(20),
            sub1_, sub2_, sub3_);
        sync_->registerCallback(&CameraSyncNode::syncCallback, this);

        // 确保保存图像的目录存在，如果不存在则创建
        if (!std::filesystem::exists(save_directory_))
        {
            std::filesystem::create_directories(save_directory_);
        }

        // 启动连接服务器的线程
        std::thread(&CameraSyncNode::connectToServer, this).detach();
    }

private:
    int sock;
    int save_count_; // 声明 save_count_ 变量
    void connectToServer() {
        while (rclcpp::ok()) {
            struct sockaddr_in serv_addr;

            // 创建socket
            if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Socket creation error");
                std::this_thread::sleep_for(5s);
                continue;
            }

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(server_port);

            // 将IP地址转换为二进制形式
            if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported");
                close(sock);
                std::this_thread::sleep_for(5s);
                continue;
            }

            // 尝试连接到服务器
            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Connection failed");
                close(sock);
                std::this_thread::sleep_for(5s);
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully connected to server");

            // 对不同命令的处理
            while (rclcpp::ok()) {
                char buffer[1024] = {0};
                int valread = read(sock, buffer, 1024);
                if (valread > 0) {
                    std::string command(buffer, valread);
                    if (command.rfind("capture", 0) == 0) {
                        // 提取照片张数
                        int num_images = save_count_;           // 默认捕获 20 张
                        if (command.length() > 7) {
                            num_images = std::stoi(command.substr(8));
                        }
                        setSaveCount(num_images);  // 设置保存数量
                        // 打开摄像头并建立 ROS2 发布节点
                        system("ros2 run two_camera appsink1 & echo $! > /tmp/appsink1.pid");
                        system("ros2 run two_camera appsink2 & echo $! > /tmp/appsink2.pid");
                        system("ros2 run two_camera appsink3 & echo $! > /tmp/appsink3.pid");
                        RCLCPP_INFO(this->get_logger(), "Executing capture command");

                        capture_flag_ = true;               // 启动捕获模式
                        image_count_ = 0;                   // 重置保存图片的计数器
                        std::this_thread::sleep_for(5s);    // 等待图像保存完成

                    } else if (command == "save") {
                        RCLCPP_INFO(this->get_logger(), "Executing save command");
                        sendAllImagesToServer();            // 发送所有保存的图像到服务器
                    } else if (command == "shutdown") {
                        RCLCPP_INFO(this->get_logger(), "Shutting down");
                        system("fuser -k /dev/video0");     // 读取 .pid 文件并杀掉进程
                        system("fuser -k /dev/video2");
                        system("fuser -k /dev/video4");
                        RCLCPP_INFO(this->get_logger(), "Camera stopped, ROS nodes terminated.");
                        
                        deleteLocalImages();                // 删除本地保存的图片

                        capture_flag_ = false;              // 停止捕获模式
                        close(sock);
                        // rclcpp::shutdown();              // 注释掉这两行就可以一直打开树莓派程序
                        // return;
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Disconnected from server, reconnecting...");
                    close(sock);
                    break;
                }
            }
        }
    }

// 删除本地保存的图片
void deleteLocalImages() {
    if (std::filesystem::exists(save_directory_)) {
        for (const auto& entry : std::filesystem::directory_iterator(save_directory_)) {
            if (entry.is_regular_file()) {
                try {
                    std::filesystem::remove(entry.path());
                    RCLCPP_INFO(this->get_logger(), "Deleted image: %s", entry.path().c_str());
                } catch (const std::filesystem::filesystem_error& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error deleting file %s: %s", entry.path().c_str(), e.what());
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "All images deleted.");
    } else {
        RCLCPP_WARN(this->get_logger(), "Save directory does not exist: %s", save_directory_.c_str());
    }
}

// 发送单张图片
void sendImageToServer(const std::string& img_path) {
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);       // 读取保存路径下的图片
    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read image: %s", img_path.c_str());
        return;
    }
    // 发送文件名长度和文件名
    std::string filename = std::filesystem::path(img_path).filename().string();
    int filename_size = filename.size();
    std::string filename_length_str = std::to_string(filename_size);
    filename_length_str.insert(filename_length_str.begin(), 4 - filename_length_str.length(), ' ');

    if (send(sock, filename_length_str.c_str(), 4, 0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send filename length for: %s", img_path.c_str());
        return;
    }

    if (send(sock, filename.c_str(), filename_size, 0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send filename for: %s", img_path.c_str());
        return;
    }
    // 编码图片
    std::vector<uchar> img_encode;                              // 编码图片
    if (!cv::imencode(".jpg", img, img_encode)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to encode image: %s", img_path.c_str());
        return;
    }

    int img_size = img_encode.size();
    const uchar* img_data = img_encode.data();

    std::string length_str = std::to_string(img_size);
    length_str.insert(length_str.begin(), 16 - length_str.length(), ' ');

    RCLCPP_INFO(this->get_logger(), "Sending image size: %d", img_size);
    if (send(sock, length_str.c_str(), 16, 0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send image size for: %s", img_path.c_str());
        return;
    }

    for (int i = 0; i < img_size; i += 1024) {
        int bytes_sent = send(sock, img_data + i, std::min(1024, img_size - i), 0);
        if (bytes_sent == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send image data for: %s", img_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Sent %d bytes of image data", bytes_sent);
    }

    char buffer[1024] = {0};
    int bytes_received = recv(sock, buffer, sizeof(buffer), 0);
    if (bytes_received == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive acknowledgment for: %s", img_path.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received acknowledgment: %s", buffer);

    if (std::string(buffer).find("ok") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "Sent image: %s successfully", img_path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send image: %s", img_path.c_str());
    }
}

void sendAllImagesToServer() {
    // 遍历保存图像的目录
    for (const auto &entry : std::filesystem::directory_iterator(save_directory_)) {
        if (entry.is_regular_file()) {              // 检查是否是常规文件（即非目录等其他类型）
            std::string file_path = entry.path().string();

            RCLCPP_INFO(this->get_logger(), "Sending image: %s", file_path.c_str());
            sendImageToServer(file_path);
        } else {
            RCLCPP_WARN(this->get_logger(), "Skipped non-file entry: %s", entry.path().string().c_str());
        }
    }
}
void setSaveCount(int count) {
    save_count_ = count;
}
// 同步回调函数，用于处理同步后的图像
void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image1,
                      const sensor_msgs::msg::Image::ConstSharedPtr& image2,
                      const sensor_msgs::msg::Image::ConstSharedPtr& image3)
    {
        // 如果未接收到 capture 命令，跳过处理
        if (!capture_flag_) {
            return;
        }

        // 如果已经保存了足够多的图像，停止捕获
        if (image_count_ >= save_count_) {
            capture_flag_ = false;
            return;
        }
        // 将每个时间戳转换为总纳秒数
        int64_t nanosec1 = image1->header.stamp.sec * 1000000000LL + image1->header.stamp.nanosec;
        int64_t nanosec2 = image2->header.stamp.sec * 1000000000LL + image2->header.stamp.nanosec;
        int64_t nanosec3 = image3->header.stamp.sec * 1000000000LL + image3->header.stamp.nanosec;

        // 计算总纳秒的平均值，平均总纳秒值转换回纳秒和秒
        int64_t avg_nanosec = (nanosec1 + nanosec2 + nanosec3) / 3;
        int64_t avg_sec = avg_nanosec / 1000000000LL;
        avg_nanosec = avg_nanosec % 1000000000LL;

        // 创建平均时间戳，将时间戳转换为字符串
        rclcpp::Time avg_time(avg_sec, avg_nanosec);
        std::string timestamp_str = std::to_string(avg_time.seconds());
        
        cv_bridge::CvImagePtr cv_ptr1, cv_ptr2, cv_ptr3;

        // 将 ROS 图像消息转换为 OpenCV 图像
        try
        {
            cv_ptr1 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
            cv_ptr2 = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::BGR8);
            cv_ptr3 = cv_bridge::toCvCopy(image3, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image1_cv = cv_ptr1->image;
        cv::Mat image2_cv = cv_ptr2->image;
        cv::Mat image3_cv = cv_ptr3->image;

        // 创建一个与其他图像尺寸相同的黑色图像
        cv::Mat black_image(image1_cv.rows, image1_cv.cols, image1_cv.type(), cv::Scalar(0, 0, 0));

        // 将图像拼接成 2x2 矩阵
        cv::Mat top_row, bottom_row, stitched_image;
        cv::hconcat(image1_cv, image2_cv, top_row);    // 拼接第一排图像
        cv::hconcat(image3_cv, black_image, bottom_row); // 拼接第二排图像（右下角是黑色图像）
        cv::vconcat(top_row, bottom_row, stitched_image); // 拼接最终图像

        // 保存拼接图像
        std::string file_name = save_directory_ + "/" STR(server_port) "_" +  timestamp_str + ".jpg";
        cv::imwrite(file_name, stitched_image);

        RCLCPP_INFO(this->get_logger(), "Saved image %d to %s", image_count_, file_name.c_str());
        image_count_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_image_publisher_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub1_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub2_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub3_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;

    std::atomic<bool> capture_flag_;
    int image_count_;
    std::string save_directory_;
    std::string camera1_topic_;
    std::string camera2_topic_;
    std::string camera3_topic_;
    std::string stitched_image_topic_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSyncNode>());
    rclcpp::shutdown();
    return 0;
}
