#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace netft
{

class FtListener : public rclcpp::Node
{
public:
    FtListener(const std::string& node_name, const std::string& topic_name);

    // Callback for subscription
    void data_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    // Accessor for the latest message
    geometry_msgs::msg::WrenchStamped get_current_msg() const;

private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
    geometry_msgs::msg::WrenchStamped current_msg_;
};

} // namespace netft