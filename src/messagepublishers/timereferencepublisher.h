
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#ifndef TIMEREFERENCEPUBLISHER_H
#define TIMEREFERENCEPUBLISHER_H

#include <sensor_msgs/msg/time_reference.hpp>

#include "packetcallback.h"

class TimeReferencePublisher final : public PacketCallback {
 public:
  explicit TimeReferencePublisher(rclcpp::Node::SharedPtr node) : PacketCallback{node} {
    pub_ = node->create_publisher<sensor_msgs::msg::TimeReference>("imu/time_ref", QoS());
  }

  void operator()(const XsDataPacket& packet, const rclcpp::Time& timestamp) final {
    if (packet.containsSampleTimeFine()) {
      const std::uint32_t sample_time_fine_hz = 10000U;
      const std::uint32_t one_ghz = 1000000000U;

      sensor_msgs::msg::TimeReference msg;

      std::uint32_t t_fine = packet.sampleTimeFine();
      std::uint32_t sec = t_fine / sample_time_fine_hz;
      std::uint32_t nsec = (t_fine % sample_time_fine_hz) * (one_ghz / sample_time_fine_hz);

      if (packet.containsSampleTimeCoarse()) {
        sec = packet.sampleTimeCoarse();
      }

      rclcpp::Time sample_time{static_cast<std::int32_t>(sec), nsec};

      msg.header.stamp = timestamp;
      // msg.header.frame_id = unused
      msg.time_ref = sample_time;
      // msg.source = optional

      pub_->publish(msg);
    }
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_;
};

#endif
