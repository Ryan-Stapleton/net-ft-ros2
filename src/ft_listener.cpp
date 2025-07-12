#include "netft_rdt_driver/ft_listener.hpp"

namespace netft
{

FtListener::FtListener(const std::string& node_name, const std::string& topic_name)
    : rclcpp::Node(node_name)
{
    sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic_name,
        10,
        std::bind(&FtListener::data_callback, this, std::placeholders::_1)
    );
}

void FtListener::data_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    current_msg_ = *msg;
}

geometry_msgs::msg::WrenchStamped FtListener::get_current_msg() const
{
    return current_msg_;
}

} // namespace netft