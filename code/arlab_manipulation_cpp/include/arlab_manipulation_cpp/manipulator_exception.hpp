#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_core/moveit/utils/moveit_error_code.hpp>
#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include "arlab_common_interfaces/action/orchestrator_action.hpp"

class ManipulationException : public std::exception
{
public:
  explicit ManipulationException(int code);
  explicit ManipulationException(int code, std::string &&msg);
  explicit ManipulationException(const moveit::core::MoveItErrorCode &code);
  const char *what() const noexcept override;
  int code() const noexcept { return code_; }
  std::string errorMessageFromCode(int code);

private:
  int code_;
  std::string msg_;
};
