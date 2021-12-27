#include "xda_interface.h"

#include <iostream>

#include <xscontroller/xsscanner.h>

#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/packetcallback.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/velocitypublisher.h"

#ifndef XSENS_USE_XDA
Journaller* gJournal = nullptr;
#endif

XdaInterface::XdaInterface(const std::string& name) : Node{name}, logger_{get_logger()}, xda_callback_{get_clock()} {
  DeclareParameters();
  CreateController();

  RCLCPP_INFO_STREAM(logger_, "xda_mti_driver with node name /" << get_name() << " started");
}

XdaInterface::~XdaInterface() {
  RCLCPP_INFO(logger_, "cleaning up ...");
  Close();
}

void XdaInterface::SpinFor(const std::chrono::milliseconds& timeout) {
  auto ros_packet = xda_callback_.Next(timeout);

  if (!ros_packet.second.empty())
    for (auto& cb : callbacks_) {
      cb->operator()(ros_packet.second, ros_packet.first);
    }
}

bool XdaInterface::Prepare() {
  assert(device_ != nullptr);

  if (!device_->gotoConfig())
    return HandleError("could not go to config");

#ifndef XSENS_USE_XDA
  /* read EMTS and device config stored in .mtb file header. */
  if (!device_->readEmtsAndDeviceConfiguration())
    return HandleError("could not read device configuration");
#endif

  RCLCPP_INFO(logger_, "measuring ...");
  if (!device_->gotoMeasurement())
    return HandleError("could not put device into measurement mode");

  if (rclcpp::Parameter logfile_param; get_parameter("logfile", logfile_param)) {
    const std::string& logfile = logfile_param.as_string();
    RCLCPP_INFO_STREAM(logger_, "found logfile parameter: " << logfile);

    if (device_->createLogFile(logfile) != XRV_OK)
      return HandleError("failed to create a log file! (" + logfile + ")");
    RCLCPP_INFO(logger_, "logging to %s...", logfile.c_str());

    if (!device_->startRecording())
      return HandleError("could not start recording");
  }

  return true;
}

bool XdaInterface::Connect() {
  /* Read baudrate parameter if set */
  XsBaudRate baud = XBR_Invalid;

  if (rclcpp::Parameter baud_param; get_parameter("baud", baud_param)) {
    auto baud_param_val = baud_param.as_int();
    baud = XsBaud::numericToRate(static_cast<std::int32_t>(baud_param_val));
    RCLCPP_INFO_STREAM(logger_, "found baud parameter: " << baud_param_val);
  }

  /* Read device ID parameter */
  bool check_device_id = false;
  std::string device_id;
  if (rclcpp::Parameter device_id_param; get_parameter("device_id", device_id_param)) {
    device_id = device_id_param.as_string();
    check_device_id = true;
    RCLCPP_INFO_STREAM(logger_, "found device ID parameter: " << device_id);
  }

  // Read port parameter if set
  XsPortInfo mti_port;
  if (rclcpp::Parameter port_name_param; get_parameter("port", port_name_param)) {
    const auto& port_name = port_name_param.as_string();
    mti_port = XsPortInfo(port_name, baud);

    RCLCPP_INFO_STREAM(logger_, "found port name parameter: " << port_name);
    RCLCPP_INFO_STREAM(logger_, "scanning port " << port_name << "...");

    if (!XsScanner::scanPort(mti_port, baud))
      return HandleError("no MTi device found. Verify port and baudrate.");

    if (check_device_id && mti_port.deviceId().toString() != device_id)
      return HandleError("no MTi device found with matching device ID.");

  } else {
    RCLCPP_INFO(logger_, "scanning for devices...");
    XsPortInfoArray port_infos = XsScanner::scanPorts(baud);

    for (auto const& port_info : port_infos) {
      if (port_info.deviceId().isMti() || port_info.deviceId().isMtig()) {
        if (check_device_id) {
          if (port_info.deviceId().toString() == device_id) {
            mti_port = port_info;
            break;
          }
        } else {
          mti_port = port_info;
          break;
        }
      }
    }
  }

  if (mti_port.empty())
    return HandleError("no MTi device found.");

  RCLCPP_INFO(logger_, "found a device with ID: %s @ port: %s, baudrate: %d",
              mti_port.deviceId().toString().toStdString().c_str(), mti_port.portName().toStdString().c_str(),
              XsBaud::rateToNumeric(mti_port.baudrate()));

  RCLCPP_INFO(logger_, "opening port %s ...", mti_port.portName().toStdString().c_str());
  if (!control_->openPort(mti_port))
    return HandleError("could not open port");

  device_ = std::shared_ptr<XsDevice>(control_->device(mti_port.deviceId()), [](XsDevice*) {});
  assert(device_ != nullptr);

  RCLCPP_INFO(logger_, "device: %s, with ID: %s opened.", device_->productCode().toStdString().c_str(),
              device_->deviceId().toString().c_str());

  device_->addCallbackHandler(&xda_callback_);

  return true;
}

void XdaInterface::Close() {
  if (device_ != nullptr) {
    device_->stopRecording();
    device_->closeLogFile();
    device_->removeCallbackHandler(&xda_callback_);
  }
  control_->closePort(port_);
}

void XdaInterface::RegisterPublishers() {
  if (get_parameter("pub_imu").as_bool())
    RegisterCallback<ImuPublisher>();
  if (get_parameter("pub_quaternion").as_bool())
    RegisterCallback<OrientationPublisher>();
  if (get_parameter("pub_acceleration").as_bool())
    RegisterCallback<AccelerationPublisher>();
  if (get_parameter("pub_angular_velocity").as_bool())
    RegisterCallback<AngularVelocityPublisher>();
  if (get_parameter("pub_mag").as_bool())
    RegisterCallback<MagneticFieldPublisher>();
  if (get_parameter("pub_dq").as_bool())
    RegisterCallback<OrientationIncrementsPublisher>();
  if (get_parameter("pub_dv").as_bool())
    RegisterCallback<VelocityIncrementPublisher>();
  if (get_parameter("pub_sampletime").as_bool())
    RegisterCallback<TimeReferencePublisher>();
  if (get_parameter("pub_temperature").as_bool())
    RegisterCallback<TemperaturePublisher>();
  if (get_parameter("pub_pressure").as_bool())
    RegisterCallback<PressurePublisher>();
  if (get_parameter("pub_gnss").as_bool())
    RegisterCallback<GnssPublisher>();
  if (get_parameter("pub_twist").as_bool())
    RegisterCallback<TwistPublisher>();
  if (get_parameter("pub_free_acceleration").as_bool())
    RegisterCallback<FreeAccelerationPublisher>();
  if (get_parameter("pub_transform").as_bool())
    RegisterCallback<TransformPublisher>();
  if (get_parameter("pub_positionLLA").as_bool())
    RegisterCallback<PositionLLAPublisher>();
  if (get_parameter("pub_velocity").as_bool())
    RegisterCallback<VelocityPublisher>();
}

void XdaInterface::DeclareParameters() {
  declare_parameter("logfile");

  /* connection params */
  declare_parameter("port");
  declare_parameter("baud");
  declare_parameter("device_id");

  /* publishing params */
  declare_parameter("publisher_queue_size", 5);
  declare_parameter("frame_id", "imu_link");
  declare_parameter("pub_imu", true);
  declare_parameter("pub_quaternion", true);
  declare_parameter("pub_acceleration", true);
  declare_parameter("pub_angular_velocity", true);
  declare_parameter("pub_mag", true);
  declare_parameter("pub_dq", true);
  declare_parameter("pub_dv", true);
  declare_parameter("pub_sampletime", true);
  declare_parameter("pub_temperature", true);
  declare_parameter("pub_pressure", true);
  declare_parameter("pub_gnss", true);
  declare_parameter("pub_twist", true);
  declare_parameter("pub_free_acceleration", true);
  declare_parameter("pub_transform", true);
  declare_parameter("pub_positionLLA", true);
  declare_parameter("pub_velocity", true);
}

void XdaInterface::CreateController() {
  RCLCPP_INFO(logger_, "creating XsControl object..");
  control_ = std::shared_ptr<XsControl>(XsControl::construct(), [](XsControl* obj) { obj->destruct(); });

#ifdef XSENS_USE_XDA
  XsVersion version;
  xdaVersion(&version);
  RCLCPP_INFO_STREAM(logger_, "using XDA version: " << version.toString().toStdString());
#endif
}

bool XdaInterface::HandleError(std::string_view error) {
  RCLCPP_ERROR_STREAM(logger_, error);
  Close();
  return false;
}
