#ifndef XDAINTERFACE_H
#define XDAINTERFACE_H

#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsportinfo.h>

#include "messagepublishers/packetcallback.h"

#include "xda_callback.h"

class XdaInterface : public rclcpp::Node {
 public:
  explicit XdaInterface(const std::string& name);
  ~XdaInterface() override;


  bool Connect();
  bool Prepare();
  void Close();
  void RegisterPublishers();
  void SpinFor(const std::chrono::milliseconds& timeout);

 private:
  rclcpp::Logger logger_;

  XdaCallback xda_callback_;
  XsPortInfo port_;

  std::shared_ptr<XsControl> control_;
  std::shared_ptr<XsDevice> device_;
  std::vector<std::shared_ptr<PacketCallback>> callbacks_;

  void DeclareParameters();
  void CreateController();
  bool HandleError(std::string_view error);

  template <typename Cb>
  void RegisterCallback() {
    static_assert(std::is_base_of_v<PacketCallback, Cb>, "Callback must be inherited from PacketCallback");
    callbacks_.emplace_back(std::make_shared<Cb>(shared_from_this()));
  }
};

#endif  // XDAINTERFACE_H
