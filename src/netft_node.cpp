/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * Simple stand-alone ROS2 node that takes data from NetFT sensor and
 * Publishes it on a ROS2 topic
 */

#include <rclcpp/rclcpp.hpp>
#include "netft_rdt_driver/netft_rdt_driver.hpp"
#include "netft_rdt_driver/netft_rdt_bias.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>
#include <tf2/LinearMath/Vector3.h>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // CLI options
  float pub_rate_hz;
  bool b_bias_on_startup;
  double rot;
  double alpha;
  tf2::Vector3 scale_F(1.0, 1.0, 1.0);
  string address;
  string frame_id;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("rate", po::value<float>(&pub_rate_hz)->default_value(100.0),
      "set publish rate (in hertz)")
    ("wrench", "publish older Wrench message type instead of WrenchStamped")
    ("bias", po::value<bool>(&b_bias_on_startup)->default_value(false),
      "if true computes the bias and subtracts it at every time step from the signal")
    ("rot", po::value<double>(&rot)->default_value(0.0),
      "rotation of the frame of reference of force torque vectors")
    ("alpha", po::value<double>(&alpha)->default_value(0.0),
      "alpha of exponential smoother, alpha in [0,1]")
    ("scale_x", po::value<double>(&scale_F[0])->default_value(1.0),
      "x-axis scale factor [-1 or 1]")
    ("scale_y", po::value<double>(&scale_F[1])->default_value(1.0),
      "y-axis scale factor [-1 or 1]")
    ("scale_z", po::value<double>(&scale_F[2])->default_value(1.0),
      "z-axis scale factor [-1 or 1]")
    ("address", po::value<string>(&address), "IP address of NetFT box")
    ("frame_id", po::value<string>(&frame_id)->default_value(""), "frame_id for output messages")
    ;

  po::positional_options_description p;
  p.add("address", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << endl;
    exit(EXIT_SUCCESS);
  }

  if (!vm.count("address")) {
    cout << desc << endl;
    cerr << "Please specify address of NetFT" << endl;
    exit(EXIT_FAILURE);
  }

  bool publish_wrench = false;
  if (vm.count("wrench")) {
    publish_wrench = true;
    std::cerr << "Publishing NetFT data as geometry_msgs::Wrench is deprecated" << std::endl;
  }

  // Create ROS2 node
  auto node = rclcpp::Node::make_shared("netft_node");

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_wrench;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;

  if (publish_wrench) {
    pub_wrench = node->create_publisher<geometry_msgs::msg::Wrench>("netft_data", 100);
  } else {
    pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>("netft_data", 100);
  }
  diag_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 2);

  // NetFT driver and bias
  auto netft = std::make_unique<netft_rdt_driver::NetFTRDTDriver>(address);
  netft_rdt_driver::NetFTRDTDriverBias bias(node, rot, scale_F, alpha);

  if (b_bias_on_startup) {
    bias.set_compute_bias(true);
  }

  // Diagnostics
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  rclcpp::Time last_diag_pub_time = node->now();
  rclcpp::Duration diag_pub_duration = rclcpp::Duration::from_seconds(1.0);

  // Main loop
  rclcpp::Rate pub_rate(pub_rate_hz);

  geometry_msgs::msg::WrenchStamped data;

  while (rclcpp::ok()) {
    if (netft->waitForNewData()) {
      netft->getData(data);
      data.header.frame_id = frame_id;
      if (publish_wrench) {
        if (pub_wrench) pub_wrench->publish(data.wrench);
      } else {
        bias.compute_bias(data.wrench);
        bias.update(data.wrench);
        if (pub) pub->publish(data);
      }
    }

    rclcpp::Time current_time = node->now();
    if ((current_time - last_diag_pub_time) > diag_pub_duration) {
      diag_array.status.clear();
      netft->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = node->now();
      if (diag_pub) diag_pub->publish(diag_array);
      last_diag_pub_time = current_time;
    }

    rclcpp::spin_some(node);
    pub_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}