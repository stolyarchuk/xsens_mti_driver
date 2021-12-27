
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

#ifndef GNSSPUBLISHER_H
#define GNSSPUBLISHER_H

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "packetcallback.h"

enum class GnssFixType : std::uint8_t { k2DFix = 2, k3DFix = 3, kGnssAndDeadReckoning = 4 };

class GnssPublisher final : public PacketCallback {
 public:
  explicit GnssPublisher(rclcpp::Node::SharedPtr node) : PacketCallback{node} {
    frame_id_ = node->get_parameter("frame_id").as_string();
    pub_ = node->create_publisher<sensor_msgs::msg::NavSatFix>("gnss", QoS());
  }

  void operator()(const XsDataPacket& packet, const rclcpp::Time& timestamp) final {
    if (packet.containsRawGnssPvtData()) {
      sensor_msgs::msg::NavSatFix msg;

      msg.header.stamp = timestamp;
      msg.header.frame_id = frame_id_;

      XsRawGnssPvtData gnss = packet.rawGnssPvtData();

      msg.latitude = static_cast<double>(gnss.m_lat) * 1e-7;
      msg.longitude = static_cast<double>(gnss.m_lon) * 1e-7;
      msg.altitude = static_cast<double>(gnss.m_height) * 1e-3;

      // Position covariance [m^2], ENU
      double sh = (static_cast<double>(gnss.m_hAcc) * 1e-3);
      double sv = (static_cast<double>(gnss.m_vAcc) * 1e-3);

      msg.position_covariance = {sh * sh, 0, 0, 0, sh * sh, 0, 0, 0, sv * sv};
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      switch (static_cast<GnssFixType>(gnss.m_fixType)) {
        case GnssFixType::k2DFix:  // fall through
        case GnssFixType::k3DFix:  // fall through
        case GnssFixType::kGnssAndDeadReckoning:
          msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          break;
        default:
          msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }
      msg.status.service = 0;  // unknown

      pub_->publish(msg);
    }
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  std::string frame_id_{kDefaultFrameId};
};

#endif
