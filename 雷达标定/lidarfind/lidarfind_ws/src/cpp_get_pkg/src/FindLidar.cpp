#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "termios.h"
#include "unistd.h"
#include <fcntl.h>
#include <chrono>
#include <string>
#include <functional>
#include <vector>

#include <algorithm>
#include <random>
#include <cmath>
#include <thread>
#include <chrono>
#include <fstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

#define ROUND_TIME 500            // 点位记录间隔
#define MIN_POINT 15              // 最少点位数量
#define MAX_COMPUTATION_NUM 1000  // 最高计算次数
#define MIN_POINT_DESTANCE 0.05f  // 点位最小间距
#define MIN_CHECK_TIME 100        // 判定静止最少次数
#define MIN_ERROR_DISTANCE 0.005f // 误差距离
using namespace std::chrono_literals;
class FindLidar : public rclcpp::Node
{
    struct SaveTFData
    {
        float x, y, z;
        float qx, qy, qz, qw;
    };
    SaveTFData lidar_tf;
    bool is_first = true;
    int marker_id_counter = 0;
public:
    FindLidar() : Node("FindLidar")
    {
        str = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(ROUND_TIME),
            std::bind(&FindLidar::timer_callback, this));

        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("tf_point", 10);
        // 初始化 2D 点（球体，但 Z = 0，看起来就是平面点）
        marker_.header.frame_id = "camera_init"; // 你显示的平面参考坐标系
        marker_.ns = "tf_point";
        marker_.type = visualization_msgs::msg::Marker::SPHERE;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.scale.x = 0.10; // 点大小
        marker_.scale.y = 0.10;
        marker_.scale.z = 0.10;

        marker_.color.r = 0.0; // 蓝点
        marker_.color.g = 0.6;
        marker_.color.b = 1.0;
        marker_.color.a = 1.0;
    }
    ~FindLidar()
    {
    }
    int check_time = 0;
    int GetTFMessage() // 获取节点TF数据
    {
        try
        {
            // 尝试查找从 "camera_init" 到 "aft_mapped" 的TF变换
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer->lookupTransform("camera_init", "aft_mapped", tf2::TimePointZero);

            marker_.header.stamp = this->now(); // 时间戳
            marker_.id = marker_id_counter++; // 每个点一个ID
            marker_.pose.position.x = transformStamped.transform.translation.x;
            marker_.pose.position.y = transformStamped.transform.translation.y;
            marker_.pose.position.z = 0.0; // 强制 2D 平面

            marker_pub_->publish(marker_); // 发布标记

            if (std::fabs(lidar_tf.x - transformStamped.transform.translation.x) <= MIN_ERROR_DISTANCE &&
                std::fabs(lidar_tf.y - transformStamped.transform.translation.y) <= MIN_ERROR_DISTANCE &&
                std::fabs(lidar_tf.z - transformStamped.transform.translation.z) <= MIN_ERROR_DISTANCE &&
                std::fabs(lidar_tf.qx - transformStamped.transform.rotation.x) <= MIN_ERROR_DISTANCE &&
                std::fabs(lidar_tf.qy - transformStamped.transform.rotation.y) <= MIN_ERROR_DISTANCE &&
                std::fabs(lidar_tf.qz - transformStamped.transform.rotation.z) <= MIN_ERROR_DISTANCE &&
                std::fabs(lidar_tf.qw - transformStamped.transform.rotation.w) <= MIN_ERROR_DISTANCE &&
                is_first == false)
            {
                check_time++;
                
                RCLCPP_INFO(this->get_logger(), " 位置未变化");
                return 1; // 位置未变化
            }
            else
            {
                lidar_tf.x = transformStamped.transform.translation.x;
                lidar_tf.y = transformStamped.transform.translation.y;
                lidar_tf.z = transformStamped.transform.translation.z;
                lidar_tf.qx = transformStamped.transform.rotation.x;
                lidar_tf.qy = transformStamped.transform.rotation.y;
                lidar_tf.qz = transformStamped.transform.rotation.z;
                lidar_tf.qw = transformStamped.transform.rotation.w;
                is_first = false;
                RCLCPP_INFO(this->get_logger(), " 位置有变化: x = %.3f , y = %.3f , z = %.3f", lidar_tf.x, lidar_tf.y, lidar_tf.z);
                return 0; // 位置有变化
            }
        }
        catch (tf2::TransformException &ex)
        {

            RCLCPP_WARN(this->get_logger(), "无法获取TF变换: %s", ex.what());
            std::this_thread::sleep_for(500ms);
            return -1; // 获取失败
        }
    }
    void SaveTFMessage() //
    {
        //SaveTFData lidar_tf;
        int check_stop = GetTFMessage();
        if (check_stop != 0)
        {
            ProcessData(check_stop);
            return;
        }
        PlaceList.push_back(lidar_tf);
        return;
    }
    void timer_callback() // 定时器，每次获取TF数据
    {
        SaveTFMessage();
    }
    void ProcessData(int check_stop) // 处理数据，平均数据
    {
        if (check_stop == 1 && check_time >= MIN_CHECK_TIME)
        {
            if (PlaceList.size() < MIN_POINT)
            {
                RCLCPP_INFO(this->get_logger(), "点位数量不足！");
                check_time = 0;
                return;
            }
            SaveTFData last_state = PlaceList.back(); // 取最后一个点位作为参考
            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(PlaceList.begin(), PlaceList.end(), gen); // 打乱点位顺序
            for (int i = 0; i < PlaceList.size(); ++i)
            {
                SaveTFData &point = PlaceList[i];
                float distance = std::sqrt(std::pow(point.x - last_state.x, 2) +
                                           std::pow(point.y - last_state.y, 2) +
                                           std::pow(point.z - last_state.z, 2));
                if (distance < MIN_POINT_DESTANCE) // 移除距离过近的点位
                {
                    PlaceList.erase(PlaceList.begin() + i);
                    --i; // 调整顺序，避免跳过下一个元素
                }
            }
            if (PlaceList.size() < MIN_POINT)
            {
                RCLCPP_INFO(this->get_logger(), "只有超近距离点位，数据不对！");
                check_time = 0;
                return;
            }
            // 取前三个点位进行计算
            int compare_line = (PlaceList.size() / 3 > MAX_COMPUTATION_NUM) ? MAX_COMPUTATION_NUM : PlaceList.size() / 3;
            for (int i = 0; i < compare_line; ++i)
            {
                random_points.assign(PlaceList.begin(), PlaceList.begin() + 3 * (i + 1));
                FindLidarOrigin();
            }
            float avg_x = 0, avg_y = 0;
            for (int i = 0; i < center.size(); ++i) // 计算平均值
            {
                avg_x += center[i].x;
                avg_y += center[i].y;
            }
            avg_x /= center.size();
            avg_y /= center.size();
            RCLCPP_INFO(this->get_logger(), "圆心位置计算完成，原点位置为：x = %.3f , y = %.3f , 距离为： %.3f", avg_x, avg_y, std::sqrt(avg_x * avg_x + avg_y * avg_y));
            center.clear();
            check_time = 0;

            // 发布圆心 Marker（大红点）
            visualization_msgs::msg::Marker center_marker;
            center_marker.header.frame_id = "camera_init";
            center_marker.header.stamp = this->now();
            center_marker.ns = "center_point";
            center_marker.id = 999; // Marker ID，不和已有蓝点冲突即可
            center_marker.type = visualization_msgs::msg::Marker::SPHERE;
            center_marker.action = visualization_msgs::msg::Marker::ADD;
            center_marker.pose.position.x = avg_x;
            center_marker.pose.position.y = avg_y;
            center_marker.pose.position.z = 0.0; // 2D 平面
            center_marker.scale.x = 0.2;         // 比蓝点大
            center_marker.scale.y = 0.2;
            center_marker.scale.z = 0.2;
            center_marker.color.r = 1.0; // 红色
            center_marker.color.g = 0.0;
            center_marker.color.b = 0.0;
            center_marker.color.a = 1.0;

            marker_pub_->publish(center_marker);

            return;
        }
        else if (check_stop == -1)
        {
            RCLCPP_INFO(this->get_logger(), "获取Lidar位置失败，停止记录");
            return;
        }
    }
    void FindLidarOrigin() // 计算原点位置
    {
        if (random_points.size() < 3)
            return;
        SaveTFData center_point;
        SaveTFData &p1 = random_points[0];
        SaveTFData &p2 = random_points[1];
        SaveTFData &p3 = random_points[2];
        float a1 = 2 * (p2.x - p1.x);
        float b1 = 2 * (p2.y - p1.y);
        float c1 = p2.x * p2.x + p2.y * p2.y - p1.x * p1.x - p1.y * p1.y;

        float a2 = 2 * (p3.x - p2.x);
        float b2 = 2 * (p3.y - p2.y);
        float c2 = p3.x * p3.x + p3.y * p3.y - p2.x * p2.x - p2.y * p2.y;

        float D = a1 * b2 - a2 * b1;
        if (std::fabs(D) < 1e-6)
            return; // 三点共线，无法确定圆心

        center_point.x = (c1 * b2 - c2 * b1) / D;
        center_point.y = (a1 * c2 - a2 * c1) / D;
        center.push_back(center_point);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    visualization_msgs::msg::Marker marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::vector<SaveTFData> PlaceList;     // 记录位置数据
    std::vector<SaveTFData> random_points; // 随机选择的点位数据*4
    std::vector<SaveTFData> center;        // 计算出的激光雷达原点位置
};
int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindLidar>());
    rclcpp::shutdown();
    return 0;
}