// -----------------------------------------------------------------------------
// File: manipulator_exception.cpp
// Package: arlab_manipulation_cpp
// Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>
//
// Implements ManipulationException, including the constructor overloads and
// the static code-to-message lookup table that maps numeric MoveIt and
// custom manipulation error codes to human-readable strings.
// -----------------------------------------------------------------------------

#include "arlab_manipulation_cpp/manipulator_exception.hpp"

#include <utility>

ManipulationException::ManipulationException(int code)
  : code_(code), msg_(ManipulationException::ErrorMessageFromCode(code)){}

ManipulationException::ManipulationException(int code, std::string&& msg)
  : code_(code), msg_(std::move(msg)) {}

ManipulationException::ManipulationException(
  const moveit::core::MoveItErrorCode& code)
  : code_(code.val), msg_(moveit::core::errorCodeToString(code)) {}

const char* ManipulationException::what() const noexcept {
  return msg_.c_str();
}

std::string ManipulationException::ErrorMessageFromCode(int code) {
  // Codes 0 to -31 mirror the MoveIt MoveItErrorCodes message definition.
  // Codes -35 and below are custom to this package.
  // 99999 is the generic unknown-failure sentinel.
  switch (code) {
    case 0:
      return "Undefined";
    case 1:
      return "Success";
    case -1:
      return "Planning failed";
    case -2:
      return "Invalid motion plan";
    case -3:
      return "Motion plan invalidated by environment change";
    case -4:
      return "Control failed";
    case -5:
      return "Unable to acquire sensor data";
    case -6:
      return "Timed out";
    case -7:
      return "Preempted";
    case -10:
      return "Start state in collision";
    case -11:
      return "Start state violates path constraints";
    case -12:
      return "Goal in collision";
    case -13:
      return "Goal violates path constraints";
    case -14:
      return "Goal constraints violated";
    case -15:
      return "Invalid group name";
    case -16:
      return "Invalid goal constraints";
    case -17:
      return "Invalid robot state";
    case -18:
      return "Invalid link name";
    case -19:
      return "Invalid object name";
    case -21:
      return "Frame transform failure";
    case -22:
      return "Collision checking unavailable";
    case -23:
      return "Robot state stale";
    case -24:
      return "Sensor info stale";
    case -25:
      return "Communication failure";
    case -26:
      return "Start state invalid";
    case -27:
      return "Goal state invalid";
    case -28:
      return "Unrecognized goal type";
    case -29:
      return "crash";
    case -30:
      return "abort";
    case -31:
      return "no IK solution";
    case -35:
      return "Unknown job command";
    case -60:
      return "Grasp action server not ready";
    case -61:
      return "Grasp goal time out";
    case -62:
      return "Invalid grasp goal";
    case -63:
      return "Grasp result time out";
    case -64:
      return "Grasp action failed";
    case 99999:
      return "Unknown failure";
    default:
      return "Unknown error code";
  }
}
