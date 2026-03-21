#ifndef ARLAB_MANIPULATION_CPP_MANIPULATOR_EXCEPTION_HPP_
#define ARLAB_MANIPULATION_CPP_MANIPULATOR_EXCEPTION_HPP_

#include <exception>
#include <string>

#include <moveit_core/moveit/utils/moveit_error_code.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include "arlab_common_interfaces/action/orchestrator_action.hpp"

// Represents a manipulation-specific exception with a numeric error code and
// a human-readable error message.
class ManipulationException : public std::exception {
  public:
    // Creates an exception from a numeric error code.
    explicit ManipulationException(int code);

    // Creates an exception from a numeric error code and a custom message.
    explicit ManipulationException(int code, std::string&& msg);

    // Creates an exception from a MoveIt error code.
    explicit ManipulationException(const moveit::core::MoveItErrorCode& code);

    // Returns the stored error message.
    const char* what() const noexcept override;

    // Returns the stored numeric error code.
    int code() const noexcept { return code_; }

    // Converts a numeric error code into a human-readable error message.
    std::string ErrorMessageFromCode(int code);

  private:
    int code_;
    std::string msg_;
};

#endif  // ARLAB_MANIPULATION_CPP_MANIPULATOR_EXCEPTION_HPP_
