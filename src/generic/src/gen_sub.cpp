#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class GenericSubscriber : public rclcpp::Node
{
public:
  GenericSubscriber() : Node("generic_subscriber")
  {
    // Declare parameter for topic name
    this->declare_parameter("topic_name", "test_topic");
    topic_name_ = this->get_parameter("topic_name").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Waiting for topic: %s to be available...", 
                topic_name_.c_str());
    
    // Create a timer to check for topic type periodically
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GenericSubscriber::check_topic_type, this));
  }

private:
  std::string topic_name_;
  std::string topic_type_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::GenericSubscription::SharedPtr generic_subscription_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  bool subscribed_ = false;
  
  void check_topic_type()
  {
    if (subscribed_) {
      return;  // Already subscribed
    }
    
    // Get list of all topics currently available
    auto topics_and_types = this->get_topic_names_and_types();
    
    // Look for our topic
    for (const auto & topic_and_types : topics_and_types) {
      if (topic_and_types.first == topic_name_) {
        if (!topic_and_types.second.empty()) {
          // Store the topic type in the member variable
          topic_type_ = topic_and_types.second[0];
          RCLCPP_INFO(this->get_logger(), "Detected topic type: %s", topic_type_.c_str());
          
          // Subscribe based on the detected type
          if (topic_type_ == "std_msgs/msg/String") {
            RCLCPP_INFO(this->get_logger(), "Subscribing to String topic"); 
            subscribe<std_msgs::msg::String>();
          } else if (topic_type_ == "std_msgs/msg/Int32") {
            subscribe<std_msgs::msg::Int32>();
          } else if (topic_type_ == "std_msgs/msg/Float32") {
            subscribe<std_msgs::msg::Float32>();
          } else if (topic_type_ == "std_msgs/msg/Bool") {
            subscribe<std_msgs::msg::Bool>();
          } else if (topic_type_ == "geometry_msgs/msg/Point") {
            subscribe<geometry_msgs::msg::Point>();
          } else if (topic_type_ == "geometry_msgs/msg/Pose") {
            subscribe<geometry_msgs::msg::Pose>();
          } else if (topic_type_ == "sensor_msgs/msg/Temperature") {
            subscribe<sensor_msgs::msg::Temperature>();
          } else if (topic_type_ == "nav_msgs/msg/Odometry") {
            subscribe<nav_msgs::msg::Odometry>();
          } else if (topic_type_ == "sensor_msgs/msg/Image") {
            subscribe<sensor_msgs::msg::Image>();
          } else if (topic_type_ == "sensor_msgs/msg/Imu") {
            subscribe<sensor_msgs::msg::Imu>();
          } else if (topic_type_ == "sensor_msgs/msg/LaserScan") {
            subscribe<sensor_msgs::msg::LaserScan>();
          } else {
            // Fall back to generic subscription if type not specifically handled
            RCLCPP_WARN(this->get_logger(), "Using generic subscription for unsupported type: %s", 
                        topic_type_.c_str());
            auto options = rclcpp::SubscriptionOptions();
            generic_subscription_ = this->create_generic_subscription(
              topic_name_,
              topic_type_,
              rclcpp::QoS(10),
              std::bind(&GenericSubscriber::generic_callback, this, std::placeholders::_1));
            
            if (generic_subscription_) {
              subscribed_ = true;
              timer_->cancel();
              RCLCPP_INFO(this->get_logger(), 
                         "Successfully subscribed to topic '%s' with type '%s' (generic)",
                         topic_name_.c_str(), topic_type_.c_str());
            }
          }
          break;
        }
      }
    }
    
    if (!subscribed_) {
      RCLCPP_INFO(this->get_logger(), 
                 "Waiting for topic '%s' to be available...", 
                 topic_name_.c_str());
    }
  }
  
  void generic_callback(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
  {
    RCLCPP_INFO(this->get_logger(), 
               "Received message on topic '%s' of type '%s' with size %zu bytes",
               topic_name_.c_str(), topic_type_.c_str(), serialized_msg->size());
  }
  
  template<typename T>
  void subscribe()
  {
    auto sub = this->create_subscription<T>(
      topic_name_,
      rclcpp::QoS(10),
      std::bind(&GenericSubscriber::callback_function<T>, this, std::placeholders::_1));
      
    subscriptions_.push_back(sub);
    subscribed_ = true;
    timer_->cancel();  // Stop checking
    RCLCPP_INFO(this->get_logger(), 
               "Successfully subscribed to topic '%s' with type '%s'",
               topic_name_.c_str(), topic_type_.c_str());
  }
  
  template<typename T>
  void callback_function(typename T::SharedPtr msg)
  {
    print_message<T>(msg);
  }

  template<typename T>
  void print_message(typename T::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Received message of type %s: %s", 
                topic_type_.c_str(), 
                msg_to_string(*msg).c_str());
  }

  template<typename T>
  std::string msg_to_string(const T & msg)
  {
    return "[Message printing not implemented for this type]";
  }
};

// Template specializations for various message types
template<>
std::string GenericSubscriber::msg_to_string<std_msgs::msg::String>(const std_msgs::msg::String & msg)
{
  return msg.data;
}

template<>
std::string GenericSubscriber::msg_to_string<std_msgs::msg::Int32>(const std_msgs::msg::Int32 & msg)
{
  return std::to_string(msg.data);
}

template<>
std::string GenericSubscriber::msg_to_string<std_msgs::msg::Float32>(const std_msgs::msg::Float32 & msg)
{
  return std::to_string(msg.data);
}

template<>
std::string GenericSubscriber::msg_to_string<std_msgs::msg::Bool>(const std_msgs::msg::Bool & msg)
{
  return msg.data ? "true" : "false";
}

template<>
std::string GenericSubscriber::msg_to_string<geometry_msgs::msg::Point>(const geometry_msgs::msg::Point & msg)
{
  return "x: " + std::to_string(msg.x) + ", y: " + std::to_string(msg.y) + ", z: " + std::to_string(msg.z);
}

template<>
std::string GenericSubscriber::msg_to_string<geometry_msgs::msg::Pose>(const geometry_msgs::msg::Pose & msg)
{
  return "position: [" + std::to_string(msg.position.x) + ", " + 
                        std::to_string(msg.position.y) + ", " + 
                        std::to_string(msg.position.z) + "], " +
         "orientation: [" + std::to_string(msg.orientation.x) + ", " + 
                           std::to_string(msg.orientation.y) + ", " + 
                           std::to_string(msg.orientation.z) + ", " + 
                           std::to_string(msg.orientation.w) + "]";
}

template<>
std::string GenericSubscriber::msg_to_string<sensor_msgs::msg::Temperature>(const sensor_msgs::msg::Temperature & msg)
{
  return "temperature: " + std::to_string(msg.temperature) + ", variance: " + std::to_string(msg.variance);
}

template<>
std::string GenericSubscriber::msg_to_string<nav_msgs::msg::Odometry>(const nav_msgs::msg::Odometry & msg)
{
  return "position: [" + std::to_string(msg.pose.pose.position.x) + ", " + 
                        std::to_string(msg.pose.pose.position.y) + ", " + 
                        std::to_string(msg.pose.pose.position.z) + "]";
}

template<>
std::string GenericSubscriber::msg_to_string<sensor_msgs::msg::Image>(const sensor_msgs::msg::Image & msg)
{
  return "image: " + std::to_string(msg.width) + "x" + std::to_string(msg.height) + 
         ", encoding: " + msg.encoding;
}

template<>
std::string GenericSubscriber::msg_to_string<sensor_msgs::msg::Imu>(const sensor_msgs::msg::Imu & msg)
{
  return "orientation: [" + std::to_string(msg.orientation.w) + ", " + 
                           std::to_string(msg.orientation.x) + ", " + 
                           std::to_string(msg.orientation.y) + ", " + 
                           std::to_string(msg.orientation.z) + "]";
}

template<>
std::string GenericSubscriber::msg_to_string<sensor_msgs::msg::LaserScan>(const sensor_msgs::msg::LaserScan & msg)
{
  return "ranges: " + std::to_string(msg.ranges.size()) + " points, " +
         "angle_min: " + std::to_string(msg.angle_min) + ", " +
         "angle_max: " + std::to_string(msg.angle_max);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GenericSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
