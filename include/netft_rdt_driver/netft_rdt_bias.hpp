#ifndef NETFT_RDT_BIAS_H_
#define NETFT_RDT_BIAS_H_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include "netft_rdt_driver/srv/string_cmd.hpp"

namespace netft_rdt_driver
{

class NetFTRDTDriverBias
{
public:
    NetFTRDTDriverBias(
        rclcpp::Node::SharedPtr node,
        double rot,
        const tf2::Vector3 &scale_F,
        double alpha,
        std::size_t num_points = 50
    );

    void update(geometry_msgs::msg::Wrench &wrench);

    void compute_bias(const geometry_msgs::msg::Wrench &wrench);

    void set_compute_bias(bool val = true);

private:
    void print_bias() const;

    void service_callback(
        const std::shared_ptr<netft_rdt_driver::srv::StringCmd::Request> request,
        std::shared_ptr<netft_rdt_driver::srv::StringCmd::Response> response
    );

    static inline double exponentialSmoothing(double current_raw_value, double last_smoothed_value, double alpha)
    {
        return alpha * current_raw_value + (1 - alpha) * last_smoothed_value;
    }

    // Member variables
    geometry_msgs::msg::Vector3  force_b_,  force_b_tmp_;
    geometry_msgs::msg::Vector3  torque_b_, torque_b_tmp_;

    geometry_msgs::msg::Wrench   wrench_tmp_;
    geometry_msgs::msg::Wrench   bias_msg_;
    double                       alpha_;

    rclcpp::Service<netft_rdt_driver::srv::StringCmd>::SharedPtr service_server_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_bias_status_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr     pub_bias_;
    std_msgs::msg::Bool                                           bias_status_;

    tf2::Vector3                scale_F_;

    tf2::Matrix3x3              Rot_;
    tf2::Vector3                tmp_;

    std::size_t                 num_points_;
    std::size_t                 count_;
    bool                        bComputeBias_;
    bool                        bSmooth_;

    rclcpp::Node::SharedPtr     node_;
};

} // namespace netft_rdt_driver

#endif //
