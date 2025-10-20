#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <functional>
#include <memory>
#include <cstring>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
constexpr uint8_t PACKET_HEADER = 0xFA;
constexpr size_t DATA_GROUPS = 3;
constexpr size_t DATA_PER_GROUP = 4;
constexpr size_t PACKET_SIZE = 1 + DATA_GROUPS * DATA_PER_GROUP;

float latest_num[3] = {0, 0, 0};
// 打开串口并配置参数
int open_serial(const std::string &device, int baudrate = B115200)
{
    int fd = -1;
    while (true)
    {
        fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd >= 0)
        {
            std::cout << "serial_OK" << device << std::endl;
            break;
        }
        else
        {
            std::cerr << "serial_Error" << device
                      << " : " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒重试
        }
    }
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    cfmakeraw(&tty);
    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

class Dispose : public rclcpp::Node
{
    // 初始化
public:
    Dispose(): Node("Dispose")
    {
        
        fd = open_serial("/dev/ttyCH341USB0", B115200);
        if (fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }
        publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("Dispose", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Dispose::process_and_publish, this));

        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    // 10ms一次
    void process_and_publish()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        if (fd < 0)
        {
            fd = open_serial("/dev/ttyCH341USB0", B115200);
            if (fd < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Fail");
                return;
            }
        }

        uint8_t num[PACKET_SIZE];
        int n = read(fd, num, PACKET_SIZE);

        if (n != PACKET_SIZE)
                {
                    RCLCPP_WARN(this->get_logger(), "Read failed or incomplete (%d bytes), reconnecting", n);
                    close(fd);
                    fd = -1;
                    return;
                }

                if (num[0] != PACKET_HEADER)
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid packet header: 0x%X", num[0]);
                    return;
                }

        if (num[0] == PACKET_HEADER)
        {
            float x,y,yaw;
            memcpy(&x,&num[1],sizeof(float)); 
            memcpy(&y,&num[5],sizeof(float)); 
            memcpy(&yaw,&num[9],sizeof(float)); 

            std_msgs::msg::Float32MultiArray msg;
            msg.data = {x, y, yaw};
            publisher->publish(msg);

            transformStamped.header.stamp = this->get_clock()->now();
            transformStamped.header.frame_id = "aft_mapped";   // 已有的坐标系
            transformStamped.child_frame_id = "new_link"; // 新的坐标系

            // 相对位置 (x,y,z)
            transformStamped.transform.translation.x = x - latest_num[0];
            transformStamped.transform.translation.y = y - latest_num[1];
            transformStamped.transform.translation.z = 0;

            // 子坐标系相对父坐标系的旋转 (四元数)
            transformStamped.transform.rotation.x = 0.0;
            transformStamped.transform.rotation.y = 0.0;
            transformStamped.transform.rotation.z = sin((yaw - latest_num[2]) / 2.0);
            transformStamped.transform.rotation.w = cos((yaw - latest_num[2]) / 2.0);

            tf_broadcaster->sendTransform(transformStamped);

            latest_num[0] = x;
            latest_num[1] = y;
            latest_num[2] = yaw;
            RCLCPP_INFO(this->get_logger(), "Dispose published: [%.3f, %.3f, %.3f]", x, y, yaw);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    int fd = {-1};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;



    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dispose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
