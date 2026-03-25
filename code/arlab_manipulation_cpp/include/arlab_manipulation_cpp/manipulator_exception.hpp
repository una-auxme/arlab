// -----------------------------------------------------------------------------
// File: manipulator_exception.hpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Defines ManipulationException, a domain-specific exception that carries
// a numeric MoveIt / manipulation error code alongside a human-readable
// message. Throw this exception from any motion component to signal a
// recoverable or unrecoverable manipulation failure to the orchestrator.
// -----------------------------------------------------------------------------

#ifndef ARLAB_MANIPULATION_CPP_MANIPULATOR_EXCEPTION_HPP_
#define ARLAB_MANIPULATION_CPP_MANIPULATOR_EXCEPTION_HPP_

#include <exception>
#include <string>

#include <moveit_core/moveit/utils/moveit_error_code.hpp>

#include "arlab_common_interfaces/msg/orchestrator_data.hpp"
#include "arlab_common_interfaces/action/orchestrator_action.hpp"

/**
 * Domain exception used throughout the manipulation stack to report failures
 * with a numeric error code and a human-readable message.
 */
class ManipulationException : public std::exception {
  public:

    /**
     * Creates an exception from a numeric error code. The message is derived
     * automatically via ErrorMessageFromCode().
     * @param code    Numeric manipulation error code.
     */
    explicit ManipulationException(int code);

    /**
     * Creates an exception from a numeric error code and a custom message.
     * @param code    Numeric manipulation error code.
     * @param msg     Human-readable description; ownership is transferred.
     */
    explicit ManipulationException(int code, std::string&& msg);

    /**
     * Creates an exception from a MoveIt error code. The numeric value and
     * the standard MoveIt error string are extracted automatically.
     * @param code    A MoveIt error code returned by planning or execution calls.
     */
    explicit ManipulationException(const moveit::core::MoveItErrorCode& code);

    /**
     * Returns the stored human-readable error message.
     * @returns Null-terminated C string valid for the lifetime of this object.
     */
    const char* what() const noexcept override;

      /**
     * Returns the stored numeric error code.
     * @returns Numeric manipulation error code.
     */
    int code() const noexcept { return code_; }

     /**
     * Converts a numeric error code into a human-readable error message.
     * Returns "Unknown error code" for any value that is not explicitly mapped.
     * @param code    Numeric manipulation error code.
     * @returns Descriptive string for the given code.
     */
    std::string ErrorMessageFromCode(int code);

  private:
    int code_;
    std::string msg_;
};

#endif  // ARLAB_MANIPULATION_CPP_MANIPULATOR_EXCEPTION_HPP_
