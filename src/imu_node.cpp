#include <sstream>
#include "imu/imu_cmd.h"
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

serial::Serial com; //Declare serial port object
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuMsg_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomMsg_pub;

//------------------------------------------------------------------------------
// Description: Serial port sends data to the device
// Input: buf[Len]=content to be sent
// Return: Returns the number of bytes sent
//------------------------------------------------------------------------------
int UART_Write(const U8 *buf, int Len)
{
    return com.write(buf, Len); //Send serial port data
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");//Set encoding to prevent Chinese garbled characters
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_node");

    node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    node->declare_parameter<int>("rate", 60);
    node->declare_parameter<int>("report_tag", 0x00ff);
    node->declare_parameter<int>("still_to_zero", 0);
    node->declare_parameter<bool>("use_compass", true);

    std::string port_name;
    int rate_hz;
    int report_tag;
    int still_to_zero;
    bool use_compass;

    node->get_parameter("port_name", port_name);
    node->get_parameter("rate", rate_hz);
    node->get_parameter("report_tag", report_tag);
    node->get_parameter("still_to_zero", still_to_zero);
    node->get_parameter("use_compass", use_compass);

    rate_hz = std::max(1, std::min(250, rate_hz));
    report_tag = std::max(0, std::min(0xffff, report_tag));
    still_to_zero = std::max(0, std::min(255, still_to_zero));

    RCLCPP_INFO(node->get_logger(), "IMU rate: %d Hz, report tag: 0x%04X", rate_hz, report_tag);

    imuMsg_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 20);
    odomMsg_pub = node->create_publisher<nav_msgs::msg::Odometry>("imu/odom", 20);

    try
    {
        com.setPort(port_name);
        com.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        com.setTimeout(to);
        com.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Unable to open port-->" << port_name);
        return -1;
    }
    if(!com.isOpen())
    {
        return -1;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Serial Port initialized");

    Cmd_12(5, still_to_zero, 0, use_compass ? 1 : 0, 2, rate_hz, 1, 3, 5,
           static_cast<U16>(report_tag));
    Cmd_03();
    Cmd_19();
    Cmd_05();

    unsigned short data_size;
    unsigned char tmpdata[4096];

    int processing_rate = std::min(rate_hz * 2, 1000);
    RCLCPP_INFO(node->get_logger(), "Processing loop rate: %d Hz", processing_rate);

    rclcpp::Rate loop_rate(processing_rate);
    while (rclcpp::ok())
    {
        if(data_size = com.available())
        {
            com.read(tmpdata, data_size);
            for(int i = 0; i < data_size; i++)
            {
                Cmd_GetPkt(tmpdata[i]);
            }
        }
        rclcpp::spin_some(node->get_node_base_interface());
        loop_rate.sleep();
    }

    return 0;
}
