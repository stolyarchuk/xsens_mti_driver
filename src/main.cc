#include <rclcpp/rclcpp.hpp>

#include "xda_interface.h"

Journaller* gJournal = 0;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<XdaInterface>("xsens_mti_driver");

  node->RegisterPublishers();

  if (!node->Connect())
    return EXIT_FAILURE;

  if (!node->Prepare())
    return EXIT_FAILURE;

  executor.add_node(node);

  while (rclcpp::ok()) {
    node->SpinFor(std::chrono::milliseconds{100});
    executor.spin_some();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
