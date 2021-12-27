#ifndef XDACALLBACK_H
#define XDACALLBACK_H

#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <xscontroller/xscallback.h>
#include <xstypes/xsdatapacket.h>

class XdaCallback : public XsCallback {
 public:
  using RosXsDataPacket = std::pair<rclcpp::Time, XsDataPacket>;

  explicit XdaCallback(rclcpp::Clock::SharedPtr clock, std::size_t max_buffer_size = 5);
  ~XdaCallback() override = default;

  RosXsDataPacket Next(const std::chrono::milliseconds& timeout);

 protected:
  void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override;

 private:
  rclcpp::Clock::SharedPtr clock_;

  std::size_t max_buffer_size_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::queue<RosXsDataPacket> buffer_;
};

#endif  // XDACALLBACK_H
