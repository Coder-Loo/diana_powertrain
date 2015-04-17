#ifndef COMMAND_LINE_CPP
#define COMMAND_LINE_CPP

#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>

#include "team_diana_lib/logging/logging.h"
#include "team_diana_lib/strings/strings.h"

using namespace boost::program_options;
using namespace Td;

void prepareParseMotorId(options_description& desc) {
  using namespace boost::program_options;
  desc.add_options()
    ("motor_id,i", value<int>()->default_value(1)->implicit_value(false)->required(true), "the CAN id of the motor to test");
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
