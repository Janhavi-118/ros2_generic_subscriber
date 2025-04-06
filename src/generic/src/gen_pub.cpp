#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class TestPublishers : public rclcpp::Node
{
public:
  TestPublishers() : Node("test_publishers"), count_(0)
  {
    // Create publishers for different message types
    string_pub_ = this->create_publisher<std_msgs::msg::String>("test_string", 10);
    int_pub_ = this->create_publisher<std_msgs::msg::Int32>("test_int", 10);
    float_pub_ = this->create_publisher<std_msgs::msg::Float32>("test_float", 10);
    bool_pub_ = this->create_publisher<std_msgs::msg::Bool>("test_bool", 10);
    point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("test_point", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("test_pose", 10);
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("test_temp", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("test_odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("test_imu", 10);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("test_scan", 10);
    
    // Timer to publish messages every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&TestPublishers::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Test publishers started - publishing 10 different message types");
  }

private:
  void timer_callback()
  {
    count_++;
    
    // String message
    auto string_msg = std_msgs::msg::String();
    string_msg.data = "Test string #" + std::to_string(count_);
    string_pub_->publish(string_msg);
    
    // Int message
    auto int_msg = std_msgs::msg::Int32();
    int_msg.data = count_;
    int_pub_->publish(int_msg);
    
    // Float message
    auto float_msg = std_msgs::msg::Float32();
    float_msg.data = count_ + 0.5f;
    float_pub_->publish(float_msg);
    
    // Bool message
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = (count_ % 2 == 0);
    bool_pub_->publish(bool_msg);
    
    // Point message
    auto point_msg = geometry_msgs::msg::Point();
    point_msg.x = static_cast<double>(count_);
    point_msg.y = static_cast<double>(count_ * 2);
    point_msg.z = static_cast<double>(count_ * 3);
    point_pub_->publish(point_msg);
    
    // Pose message
    auto pose_msg = geometry_msgs::msg::Pose();
    pose_msg.position.x = static_cast<double>(count_);
    pose_msg.position.y = static_cast<double>(count_ * 2);
    pose_msg.position.z = 0.0;
    pose_msg.orientation.w = 1.0;
    pose_msg.orientation.x = 0.0;
    pose_msg.orientation.y = 0.0;
    pose_msg.orientation.z = 0.0;
    pose_pub_->publish(pose_msg);
    
    // Temperature message
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.temperature = 20.0 + (count_ % 10);
    temp_msg.variance = 0.1;
    temp_pub_->publish(temp_msg);
    
    // Odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = static_cast<double>(count_ * 0.1);
    odom_msg.pose.pose.position.y = static_cast<double>(count_ * 0.05);
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.twist.twist.linear.x = 0.5;
    odom_msg.twist.twist.angular.z = 0.1;
    odom_pub_->publish(odom_msg);
    
    // IMU message
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = 1.0;
    imu_msg.angular_velocity.z = static_cast<double>(count_ % 5) * 0.1;
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;
    imu_pub_->publish(imu_msg);
    
    // LaserScan message
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = -1.57;
    scan_msg.angle_max = 1.57;
    scan_msg.angle_increment = 0.1;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 10.0;
    
    // Create sample ranges data
    const int num_readings = 32;
    scan_msg.ranges.resize(num_readings);
    for (int i = 0; i < num_readings; i++) {
      scan_msg.ranges[i] = 5.0 + std::sin(static_cast<double>(count_ + i) * 0.1) * 2.0;
    }
    scan_pub_->publish(scan_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published messages #%zu", count_);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Publishers for all message types
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublishers>());
  rclcpp::shutdown();
  return 0;
}
