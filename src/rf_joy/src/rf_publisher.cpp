#include <vector>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <libserial/SerialStreamBuf.h>
#include <libserial/SerialPortConstants.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using namespace LibSerial;
using namespace std;

const size_t IBUS_PACKET_SIZE = 32;
const string UART_PORT = "/dev/ttyTHS1"; // Jetson Orin NX의 UART 포트

bool verifyChecksum(const std::vector<uint8_t>& data) {
    uint16_t checksum = 0xFFFF;
    for (size_t i = 0; i < data.size()-2; i++) {
        checksum -= data[i];
    }
    uint16_t received_checksum = data[data.size()-2] | (data[data.size()-1] << 8);
    return checksum == received_checksum;
}

void portOpener(SerialPort &serial_port, rclcpp::Node::SharedPtr node){
    try {
        serial_port.Open(UART_PORT);

        serial_port.SetBaudRate(BaudRate::BAUD_115200);
        serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial_port.SetParity(Parity::PARITY_NONE);
        serial_port.SetStopBits(StopBits::STOP_BITS_1);
        serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

        if (!serial_port.IsOpen()) {
            RCLCPP_ERROR(node->get_logger(), "Cannot open port: %s", UART_PORT.c_str());
            return;
        }

        RCLCPP_INFO(node->get_logger(), "Waiting for IBUS data...");
    }
    catch (const OpenFailed& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open port %s: %s", UART_PORT.c_str(), e.what());
        throw; // Re-throw to handle in main
    }
}

//flush junk data untill meet header(0x20)
uint8_t CorrectDataFlusher(SerialPort &serial_port, rclcpp::Node::SharedPtr node){
    uint8_t byte;
    do{
        if(serial_port.IsDataAvailable()){
            serial_port.ReadByte(byte);
        }
    }
    while(byte != 32 && rclcpp::ok());
    return byte;
}

vector<uint16_t> parseIbusData(SerialPort &serial_port, rclcpp::Node::SharedPtr node) {
    vector<uint8_t> data;
    do{
        data.push_back(CorrectDataFlusher(serial_port, node));
        if(serial_port.IsDataAvailable()){
            for(int i = 0; i < 31; i++){
                uint8_t input;
                serial_port.ReadByte(input);
                data.push_back(input);
            }
        }

        if (data.size() != IBUS_PACKET_SIZE) {
            RCLCPP_WARN(node->get_logger(), "Received data size %zu does not match expected %zu", data.size(), IBUS_PACKET_SIZE);
        }
    }
    while(!verifyChecksum(data) && rclcpp::ok());


    vector<uint16_t> channels;
    for (size_t i = 0; i < 28; i += 2) { // Adjusted to parse correct range for 14 channels
        uint16_t channel_value = data[i] | (data[i+1] << 8);
        channels.push_back(channel_value);
    }
    return channels;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rf_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::UInt16MultiArray>("/rf", 10);

    SerialPort serial_port;

    try {
        portOpener(serial_port, node);
        while (rclcpp::ok()) {
            vector<uint16_t> data = parseIbusData(serial_port, node);
            if (data.size() < 14) {
                RCLCPP_WARN(node->get_logger(), "Insufficient channel data received.");
                continue;
            }

            vector<uint16_t> channels(data.begin() + 1, data.begin() + 15); // Adjusted indices for 14 channels
            std_msgs::msg::UInt16MultiArray message;

            message.data = channels;
            publisher->publish(message);

            rclcpp::spin_some(node);
        }


        serial_port.Close();
        RCLCPP_INFO(node->get_logger(), "%s is closed.", UART_PORT.c_str());

    } catch (const OpenFailed& e) {
        serial_port.Close();
        RCLCPP_ERROR(node->get_logger(), "Cannot open the port: %s, Exception: %s", UART_PORT.c_str(), e.what());
        return 1;
    } catch (const exception& e) {
        serial_port.Close();
        rclcpp::shutdown();
        RCLCPP_ERROR(node->get_logger(), "Exception caused: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
