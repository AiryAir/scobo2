#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// Define encoder value limits
const int encoder_minimum = -16192;
const int encoder_maximum = 16192;

// ROS node handle
rclcpp::Node::SharedPtr node;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_ticks_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_ticks_pub;

// Variables to store encoder counts
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Function prototypes
void leftEncoderISR();
void rightEncoderISR();

void leftEncoderISR() {
    // Handle left encoder interrupt
}

void rightEncoderISR() {
    // Handle right encoder interrupt
}

int main() {
    // Initialize ROS node
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("airduino_combined_ros2");

    // Create ROS publishers
    left_ticks_pub = node->create_publisher<std_msgs::msg::Int32>("left_ticks", 10);
    right_ticks_pub = node->create_publisher<std_msgs::msg::Int32>("right_ticks", 10);

    // Main loop
    while (rclcpp::ok()) {
        // Publish encoder values
        std_msgs::msg::Int32 left_ticks_msg;
        left_ticks_msg.data = left_ticks;
        left_ticks_pub->publish(left_ticks_msg);

        std_msgs::msg::Int32 right_ticks_msg;
        right_ticks_msg.data = right_ticks;
        right_ticks_pub->publish(right_ticks_msg);

        // Spin ROS node
        rclcpp::spin_some(node);
    }

    // Shutdown ROS node
    rclcpp::shutdown();
    return 0;
}

