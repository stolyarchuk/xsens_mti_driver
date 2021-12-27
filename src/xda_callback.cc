#include "xda_callback.h"

XdaCallback::XdaCallback(rclcpp::Clock::SharedPtr clock, std::size_t max_buffer_size)
    : clock_{std::move(clock)}
    , max_buffer_size_{max_buffer_size} {}

XdaCallback::RosXsDataPacket XdaCallback::Next(const std::chrono::milliseconds& timeout) {
  RosXsDataPacket packet;

  std::unique_lock<std::mutex> lock(mutex_);

  if (condition_.wait_for(lock, timeout, [&] { return !buffer_.empty(); })) {
    assert(!buffer_.empty());

    packet = buffer_.front();
    buffer_.pop();
  }

  return packet;
}

void XdaCallback::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) {
  std::unique_lock<std::mutex> lock{mutex_};

  assert(packet != nullptr);

  // Discard oldest packet if buffer full
  if (buffer_.size() == max_buffer_size_)
    buffer_.pop();

  // Push new packet
  buffer_.push(std::make_pair(clock_->now(), *packet));

  // Manual unlocking is done before notifying, to avoid waking up
  // the waiting thread only to block again
  lock.unlock();
  condition_.notify_one();
}
