#ifndef COMMAND_LINE_CPP
#define COMMAND_LINE_CPP

#include "diana_powertrain/command_line.hpp"

#include "team_diana_lib/logging/logging.h"
#include "team_diana_lib/strings/strings.h"

using namespace boost;
using namespace boost::program_options;
using namespace Td;

void prepareParseMotorId(option_description& desc) {
  desc.add_options()
    ("motor_id,i", value<int>()->default_value(1)->implicit_value(false), "the CAN id of the motor to test");
}

bool parseMotorId(variables_map& varsMap, int& motorId) {
  if(!varsMap["motor_id"].empty()) {
    motorId = varsMap["motor_id"].as<int>();
  } else {
    ros_error("No serials specified. Use help to see usage");
    return false;
  }
  return true;
}


#endif // COMMAND_LINE_CPP
