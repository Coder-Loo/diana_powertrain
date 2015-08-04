#ifndef COMMAND_LINE_CPP
#define COMMAND_LINE_CPP

#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "team_diana_lib/logging/logging.h"
#include "team_diana_lib/strings/strings.h"

#include <vector>

using namespace boost::program_options;
using namespace Td;
using namespace std;

void prepareParseMotorId(options_description& desc) {
  using namespace boost::program_options;
  desc.add_options()
    ("motor_id,i", value<int>()->default_value(1)->implicit_value(false)->required(), "the CAN id of the motor to test");
}

void prepareParseMotorIds(options_description& desc) {
  using namespace boost::program_options;
  desc.add_options()
    ("motor_ids,i", value<vector<string>>()->multitoken()->
        required(), "the CAN IDs of the motors to test");
}

bool isMotorId(int id) {
  return (id >= 0 && id <= 127);
}

bool parseMotorId(variables_map& varsMap, int& motorId) {
  if(!varsMap["motor_id"].empty() && !varsMap["motor_id"].defaulted()) {
    motorId = varsMap["motor_id"].as<int>();
    if(!isMotorId(motorId)) {
      ros_error("Motor id must be between >=0 and <=127");
      return false;
    }
  } else {
    ros_error("No motor id specified. Use help to see usage");
    return false;
  }
  return true;
}

bool parseMotorIds(variables_map& varsMap, std::vector<int>& motorIds) {
  if(!varsMap["motor_id"].empty() && !varsMap["motor_id"].defaulted()) {
    vector<string> motorIdsStr;
    motorIdsStr = varsMap["motor_id"].as<vector<string>>();
    for(const string& s : motorIdsStr) {
      try {
        int id = boost::lexical_cast<int>(s);
        motorIds.push_back(id);
      } catch(const boost::bad_lexical_cast& e) {
         ros_error(Td::toString("unable to parse motor id: ", s, " - ", e.what()));
      }
    }

    if(!std::all_of(motorIds.begin(), motorIds.end(), isMotorId)) {
      ros_error("Motor id must be between >=0 and <=127");
      return false;
    }
  } else {
    ros_error("No motor id specified. Use help to see usage");
    return false;
  }
  return true;
}

bool readYesNoCommandline() {
  string ans;
  while(true) {
    std::cin >> ans;
    boost::to_lower(ans);
    if(boost::starts_with(ans, "y")) {
      return true;
    } else if (boost::starts_with(ans, "n")) {
      return false;
    }
  }
}

#endif // COMMAND_LINE_CPP
