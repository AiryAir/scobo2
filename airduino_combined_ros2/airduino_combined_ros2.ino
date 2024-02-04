#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// Define encoder pins
#define ENC_IN_LEFT_A 18
#define ENC_IN_RIGHT_A 20
#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 21

// Define encoder value limits
const int encoder_minimum = -16192;
const int encoder_maximum = 16192;

// Define PWM control pins
const int leftEn = 2;
const int rightEn = 3;
const int leftBackward = 7;
const int leftForward = 6;
const int rightForward = 4;
const int rightBackward = 5;

class EncoderPwmNode : public rclcpp::Node
{
public:
    EncoderPwmNode() : Node("encoder_pwm_node")
    {
        // Advertise publishers
        left_ticks_pub = this->create_publisher<std_msgs::msg::Int32>("left_ticks", 10);
        right_ticks_pub = this->create_publisher<std_msgs::msg::Int32>("right_ticks", 10);

        // Subscribe to PWM topics
        left_pwm_sub = this->create_subscription<std_msgs::msg::Int32>("left_pwm", 10,
                                                                     std::bind(&EncoderPwmNode::leftPwmCallback, this, std::placeholders::_1));
        right_pwm_sub = this->create_subscription<std_msgs::msg::Int32>("right_pwm", 10,
                                                                      std::bind(&EncoderPwmNode::rightPwmCallback, this, std::placeholders::_1));
        
        // Initialize PWM control pins
        pinMode(leftEn, OUTPUT);
        pinMode(rightEn, OUTPUT);
        pinMode(leftBackward, OUTPUT);
        pinMode(leftForward, OUTPUT);
        pinMode(rightForward, OUTPUT);
        pinMode(rightBackward, OUTPUT);

        // Configure encoder pins as inputs
        pinMode(ENC_IN_LEFT_A, INPUT);
        pinMode(ENC_IN_LEFT_B, INPUT);
        pinMode(ENC_IN_RIGHT_A, INPUT);
        pinMode(ENC_IN_RIGHT_B, INPUT);

        // Attach interrupts for encoder pins
        attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), std::bind(&EncoderPwmNode::leftEncoderISR, this));
        attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), std::bind(&EncoderPwmNode::rightEncoderISR, this));
    }

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_ticks_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_ticks_pub;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_pwm_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_pwm_sub;

    // Variables to store encoder counts
    volatile long left_ticks = 0;
    volatile long right_ticks = 0;

    // Callback function for left PWM subscriber
    void leftPwmCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // Handle PWM control
        analogWrite(leftEn, msg->data);
        if (msg->data >= 0)
        {
            digitalWrite(leftForward, HIGH);
            digitalWrite(leftBackward, LOW);
        }
        else
        {
            digitalWrite(leftForward, LOW);
            digitalWrite(leftBackward, HIGH);
        }
    }

    // Callback function for right PWM subscriber
    void rightPwmCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // Handle PWM control
        analogWrite(rightEn, msg->data);
        if (msg->data >= 0)
        {
            digitalWrite(rightForward, HIGH);
            digitalWrite(rightBackward, LOW);
        }
        else
        {
            digitalWrite(rightForward, LOW);
            digitalWrite(rightBackward, HIGH);
        }
    }

    // Interrupt service routine for left encoder
    void leftEncoderISR()
    {
        if (digitalRead(ENC_IN_LEFT_A) == digitalRead(ENC_IN_LEFT_B))
        {
            left_ticks++;
        }
        else
        {
            left_ticks--;
        }

        // Limit encoder value within specified range
        left_ticks = constrain(left_ticks, encoder_minimum, encoder_maximum);

        // Publish encoder values
        auto msg = std::make_unique<std_msgs::msg::Int32>();
        msg->data = left_ticks;
        left_ticks_pub->publish(std::move(msg));
    }

    // Interrupt service routine for right encoder
    void rightEncoderISR()
    {
        if (digitalRead(ENC_IN_RIGHT_A) == digitalRead(ENC_IN_RIGHT_B))
        {
            right_ticks++;
        }
        else
        {
            right_ticks--;
        }

        // Limit encoder value within specified range
        right_ticks = constrain(right_ticks, encoder_minimum, encoder_maximum);

        // Publish encoder values
        auto msg = std::make_unique<std_msgs::msg::Int32>();
        msg->data = right_ticks;
        right_ticks_pub->publish(std::move(msg));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderPwmNode>());
    rclcpp::shutdown();
    return 0;
}
