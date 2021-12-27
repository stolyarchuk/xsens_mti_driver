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

  std::cout << "main1" << std::endl;
  while (rclcpp::ok()) {
    node->SpinFor(std::chrono::milliseconds{100});
    executor.spin_some();
  }

  std::cout << "main4" << std::endl;
  rclcpp::shutdown();
  std::cout << "main5" << std::endl;
  return EXIT_SUCCESS;
}
