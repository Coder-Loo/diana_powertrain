#include "diana_powertrain/consts.hpp"
#include <team_diana_lib/strings/bit_printer.h>

std::ostream& operator<< (std::ostream & os, ElmoStatus val)
{
    switch (val)
    {
        case ElmoStatus::NOT_READY_TO_SWITCH_ON : return os << "NOT_READY_TO_SWITCH_ON";
        case ElmoStatus::SWITCH_ON_DISABLED : return os << "SWITCH_ON_DISABLED";
        case ElmoStatus::READY_TO_SWITCH_ON : return os << "READY_TO_SWITCH_ON";
        case ElmoStatus::SWITCH_ON : return os << "SWITCH_ON";
        case ElmoStatus::OPERATION_ENABLED : return os << "OPERATION_ENABLED";
        case ElmoStatus::QUICK_STOP_ACTIVE : return os << "QUICK_STOP_ACTIVE";
        case ElmoStatus::FAULT_REACTION_ACTIVE : return os << "FAULT_REACTION_ACTIVE";
        case ElmoStatus::FAULT : return os << "FAULT";
        case ElmoStatus::UNKNOWN : return os << "UNKNOWN";
    };
    return os << "UNKNOWN VALUE - FIX ME";
}

uint16_t getControlWordCommandBits(ControlWordCommand command)
{
  switch(command) {
    //                                                   7   3210
    case ControlWordCommand::SHUTDOWN:          return 0b00000110;
    case ControlWordCommand::SWITCH_ON:         return 0b00000111;
    case ControlWordCommand::DISABLE_VOLTAGE:   return 0b00000000;
    case ControlWordCommand::QUICK_STOP:        return 0b00000010;
    case ControlWordCommand::DISABLE_OPERATION: return 0b00000111;
    case ControlWordCommand::ENABLE_OPERATION:  return 0b00001111;
    case ControlWordCommand::FAULT_RESET:       return 0b10000000;
  }
  return 0;
}


bool getControlWordCommandFromString(const std::string s, ControlWordCommand& val) {
  if (s == "SHUTDOWN")  val = ControlWordCommand::SHUTDOWN;
  else if (s == "SWITCH_ON") val = ControlWordCommand::SWITCH_ON;
  else if (s == "DISABLE_VOLTAGE") val = ControlWordCommand::DISABLE_VOLTAGE;
  else if (s == "QUICK_STOP") val = ControlWordCommand::QUICK_STOP;
  else if (s == "DISABLE_OPERATION") val = ControlWordCommand::DISABLE_OPERATION;
  else if (s == "ENABLE_OPERATION") val = ControlWordCommand::ENABLE_OPERATION;
  else if (s == "FAULT_RESET") val = ControlWordCommand::FAULT_RESET;
  else return false;
  return true;
}

std::ostream& operator<< (std::ostream & os, ModeOfOperation val)
{
    Td::BitPrinter<4> bitPrinter;
    os << bitPrinter.toString((int)val) << " ";
    switch (val)
    {
        case NO_MODE                : return os << "NO_MODE";
        case PROFILE_POSITION       : return os << "PROFILE_POSITION";
        case PROFILED_VELOCITY      : return os << "PROFILED_VELOCITY";
        case TORQUE_PROFILED        : return os << "TORQUE_PROFILED";
        case HOMING                 : return os << "HOMING";
        case INTERPOLATION_POSITION : return os << "INTERPOLATION_POSITION";
    };
    return os << "UNKNOWN VALUE - FIX ME - CHECK WARNINGS";
}


bool getModeOfOperationFromString(const std::string s, ModeOfOperation& val) {
  if (s == "NO_MODE")  val = ModeOfOperation::NO_MODE;
  else if (s == "PROFILE_POSITION") val = ModeOfOperation::PROFILE_POSITION;
  else if (s == "PROFILED_VELOCITY") val = ModeOfOperation::PROFILED_VELOCITY;
  else if (s == "TORQUE_PROFILED") val = ModeOfOperation::TORQUE_PROFILED;
  else if (s == "HOMING") val = ModeOfOperation::HOMING;
  else if (s == "INTERPOLATION_POSITION") val = ModeOfOperation::INTERPOLATION_POSITION;
  else return false;
  return true;
}
