#ifndef PAYLOAD_PARAMETERS_HPP
#define PAYLOAD_PARAMETERS_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace payload {

struct Params {
  std::string controller = "SimController";
};

// Lightweight local replacement for generate_parameter_library.
class ParamListener {
 public:
  explicit ParamListener(rclcpp::Node* node) : node_(node) {
    declare_if_missing("controller", Params{}.controller);
  }

  explicit ParamListener(const std::shared_ptr<rclcpp::Node>& node)
      : ParamListener(node.get()) {}

  Params get_params() const {
    Params params;
    params.controller = node_->get_parameter("controller").as_string();
    return params;
  }

 private:
  template <typename T>
  void declare_if_missing(const std::string& name, const T& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
  }

  rclcpp::Node* node_;
};

}  // namespace payload

#endif
