#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <vector>

class JoyPublisher : public rclcpp::Node
{
public:
    JoyPublisher() : Node("joy_publisher")
    {
        joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
        fr_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/rf", 10, std::bind(&JoyPublisher::fr_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "rf to joy node starts");
        
    }

private:
    void fr_callback(const std_msgs::msg::UInt16MultiArray msg)
    {
        auto joy_msg = sensor_msgs::msg::Joy();

        if(msg.data[4]>1600){                //kill switch
            if(msg.data[5]<1600){ // user input mode
                joy_msg.axes.push_back((msg.data[0]-1500)/500.0);
                joy_msg.axes.push_back((msg.data[1]-1500)/500.0);
                joy_msg.axes.push_back((msg.data[2]-1500)/500.0);
                joy_msg.axes.push_back((msg.data[3]-1500)/500.0);
                joy_msg.buttons.push_back((msg.data[5] > 1300 )? 1 : 0);
                joy_msg.buttons.push_back((msg.data[5]>1600)? 1:0);
                joy_publisher_->publish(joy_msg);
            }
            //else{} // autonomus driving(no joy topic publishing)
        }
        else{ //kill switch(default input setting)
            joy_msg.axes.push_back(0);
            joy_msg.axes.push_back(0);
            joy_msg.axes.push_back(0);
            joy_msg.axes.push_back(0);
            joy_msg.buttons.push_back(0);
            joy_msg.buttons.push_back(0);
            joy_publisher_->publish(joy_msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr fr_subscriber_;
    int fail_counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyPublisher>());

    rclcpp::shutdown();
    return 0;
}