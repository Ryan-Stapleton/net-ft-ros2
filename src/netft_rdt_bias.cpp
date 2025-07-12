#include "netft_rdt_driver/netft_rdt_bias.hpp"

namespace netft_rdt_driver
{

NetFTRDTDriverBias::NetFTRDTDriverBias(
    rclcpp::Node::SharedPtr node,
    double rot,
    const tf2::Vector3& scale_F,
    double alpha,
    std::size_t num_points)
: node_(node),
  num_points_(num_points),
  scale_F_(scale_F),
  alpha_(alpha)
{
  count_      = 0;
  force_b_.x  = 0;
  force_b_.y  = 0;
  force_b_.z  = 0;
  torque_b_.x = 0;
  torque_b_.y = 0;
  torque_b_.z = 0;

  force_b_tmp_  = force_b_;
  torque_b_tmp_ = torque_b_;

  bSmooth_ = (alpha_ != 0.0);

  service_server_ = node_->create_service<netft_rdt_driver::srv::StringCmd>(
      "bias_cmd",
      std::bind(&NetFTRDTDriverBias::service_callback, this,
                std::placeholders::_1, std::placeholders::_2)
  );
  pub_bias_status_ = node_->create_publisher<std_msgs::msg::Bool>("bias_status", 1);
  pub_bias_        = node_->create_publisher<geometry_msgs::msg::Wrench>("bias", 1);
  bias_status_.data = false; // bias not set
  bComputeBias_     = false;

  Rot_.setRPY(0, 0, rot);
}

void NetFTRDTDriverBias::update(geometry_msgs::msg::Wrench& wrench)
{
  // filter the signal
  if(bSmooth_) {
    wrench_tmp_.force.x  = exponentialSmoothing(wrench.force.x,  wrench_tmp_.force.x,  alpha_);
    wrench_tmp_.force.y  = exponentialSmoothing(wrench.force.y,  wrench_tmp_.force.y,  alpha_);
    wrench_tmp_.force.z  = exponentialSmoothing(wrench.force.z,  wrench_tmp_.force.z,  alpha_);
    wrench_tmp_.torque.x = exponentialSmoothing(wrench.torque.x, wrench_tmp_.torque.x, alpha_);
    wrench_tmp_.torque.y = exponentialSmoothing(wrench.torque.y, wrench_tmp_.torque.y, alpha_);
    wrench_tmp_.torque.z = exponentialSmoothing(wrench.torque.z, wrench_tmp_.torque.z, alpha_);
    wrench = wrench_tmp_;
  }

  // remove the bias
  wrench.force.x  -= force_b_.x;
  wrench.force.y  -= force_b_.y;
  wrench.force.z  -= force_b_.z;
  wrench.torque.x -= torque_b_.x;
  wrench.torque.y -= torque_b_.y;
  wrench.torque.z -= torque_b_.z;

  bias_msg_.force.x  = force_b_.x;
  bias_msg_.force.y  = force_b_.y;
  bias_msg_.force.z  = force_b_.z;
  bias_msg_.torque.x = torque_b_.x;
  bias_msg_.torque.y = torque_b_.y;
  bias_msg_.torque.z = torque_b_.z;

  // Rotate the force vector
  tmp_[0] = wrench.force.x;
  tmp_[1] = wrench.force.y;
  tmp_[2] = wrench.force.z;
  tmp_    = Rot_ * tmp_;
  wrench.force.x = tmp_[0] * scale_F_[0];
  wrench.force.y = tmp_[1] * scale_F_[1];
  wrench.force.z = tmp_[2] * scale_F_[2];

  // Rotate the torque vector
  tmp_[0] = wrench.torque.x;
  tmp_[1] = wrench.torque.y;
  tmp_[2] = wrench.torque.z;
  tmp_    = Rot_ * tmp_;
  wrench.torque.x = tmp_[0];
  wrench.torque.y = tmp_[1];
  wrench.torque.z = tmp_[2];

  pub_bias_status_->publish(bias_status_);
  pub_bias_->publish(bias_msg_);
}

void NetFTRDTDriverBias::set_compute_bias(bool val)
{
  bComputeBias_ = val;
}

void NetFTRDTDriverBias::compute_bias(const geometry_msgs::msg::Wrench& wrench)
{
  if(bComputeBias_) {
    if(count_ < num_points_) {
      force_b_tmp_.x  += wrench.force.x;
      force_b_tmp_.y  += wrench.force.y;
      force_b_tmp_.z  += wrench.force.z;
      torque_b_tmp_.x += wrench.torque.x;
      torque_b_tmp_.y += wrench.torque.y;
      torque_b_tmp_.z += wrench.torque.z;
      count_++;
    } else {
      force_b_tmp_.x /= static_cast<double>(num_points_);
      force_b_tmp_.y /= static_cast<double>(num_points_);
      force_b_tmp_.z /= static_cast<double>(num_points_);
      torque_b_tmp_.x /= static_cast<double>(num_points_);
      torque_b_tmp_.y /= static_cast<double>(num_points_);
      torque_b_tmp_.z /= static_cast<double>(num_points_);

      force_b_  = force_b_tmp_;
      torque_b_ = torque_b_tmp_;

      print_bias();
      bComputeBias_     = false;
      bias_status_.data = true;
    }
  }
}

void NetFTRDTDriverBias::print_bias() const
{
  std::cout << "=== bias (mean) ===" << std::endl;
  std::cout << "F:         " << force_b_.x << "\t" << force_b_.y << "\t" << force_b_.z << std::endl;
  std::cout << "T:         " << torque_b_.x << "\t" << torque_b_.y << "\t" << torque_b_.z << std::endl;
  std::cout << "nbSamples: " << num_points_ << std::endl;
}

// ROS2 service callback
void NetFTRDTDriverBias::service_callback(
    const std::shared_ptr<netft_rdt_driver::srv::StringCmd::Request> request,
    std::shared_ptr<netft_rdt_driver::srv::StringCmd::Response> response)
{
  std::string cmd = request->cmd;

  if(cmd == "bias") {
    bComputeBias_ = true;
    bias_status_.data = false;
    force_b_tmp_.x = force_b_tmp_.y = force_b_tmp_.z = 0;
    torque_b_tmp_.x = torque_b_tmp_.y = torque_b_tmp_.z = 0;
    count_ = 0;
    response->res = " command [" + cmd + "] successfully called";
  } else if (cmd == "print") {
    response->res = "commands : bias | print";
  } else {
    response->res = "no such cmd [" + cmd + "] defined        NetFTRDTDriverBias::service_callback";
  }
}

} // namespace netft_rdt_driver
