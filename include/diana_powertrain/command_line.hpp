#ifndef COMMAND_LINE_HPP
#define COMMAND_LINE_HPP

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include <boost/program_options.hpp>

void prepareParseMotorId(boost::program_options::options_description& desc);
void prepareParseMotorIds(boost::program_options::options_description& desc);
bool parseMotorId(boost::program_options::variables_map& varsMap, int& motorId);
bool parseMotorIds(boost::program_options::variables_map& varsMap, std::vector<int>& motorId);
bool readYesNoCommandline();

#endif // COMMAND_LINE_HPP
