
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

#ifndef MAGNETICFIELDPUBLISHER_H
#define MAGNETICFIELDPUBLISHER_H

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "packetcallback.h"

class MagneticFieldPublisher final : public PacketCallback {
 public:
  explicit MagneticFieldPublisher(rclcpp::Node::SharedPtr node) : PacketCallback{node} {
    frame_id_ = node->get_parameter("frame_id").as_string();
    pub_ = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/mag", QoS());
    // variance_from_stddev_param("~magnetic_field_stddev", magnetic_field_variance);
  }

  void operator()(const XsDataPacket& packet, const rclcpp::Time& timestamp) final {
    if (packet.containsCalibratedMagneticField()) {
      // TODO: Use sensor_msgs::MagneticField
      // Problem: Sensor gives normalized magnetic field vector with unknown units
      geometry_msgs::msg::Vector3Stamped msg;

      msg.header.stamp = timestamp;
      msg.header.frame_id = frame_id_;

      XsVector mag = packet.calibratedMagneticField();

      msg.vector.x = mag[0];
      msg.vector.y = mag[1];
      msg.vector.z = mag[2];

      // msg.magnetic_field_covariance[0] = magnetic_field_variance[0];
      // msg.magnetic_field_covariance[4] = magnetic_field_variance[1];
      // msg.magnetic_field_covariance[8] = magnetic_field_variance[2];

      pub_->publish(msg);
    }
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_;
  std::string frame_id_{kDefaultFrameId};

  // double magnetic_field_variance[3];
};

#endif
