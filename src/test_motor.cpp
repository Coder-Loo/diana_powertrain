#include <iostream>
#include <string>

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include <boost/program_options.hpp>

INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;
using namespace std;
using namespace Td;

bool parseCommandLine(int argc, char** argv, int& motorId) {
  using namespace boost::program_options;
  options_description desc("Options");
  desc.add_options()
    ("help,h", "Print help messages")
    ("motor_id,i", value<int>()->default_value(1)->implicit_value(false), "the CAN id of the motor to test");
  variables_map varsMap;

  try {
      store(parse_command_line(argc, argv, desc), varsMap);

      if(varsMap.count("help")) {
        cout << desc << endl;
        return false;
      }

  } catch (error& e) {
      ros_error(toString("Unable to parse description: ",  e.what()));
      return false;
  }

  return true;
}

int main(int argc, char** argv) {
  Pci7841Card card(0, 0);
  hlcanopen::CanOpenManager<Pci7841Card> canOpenManager(card);

  int motorId = 0;
  parseCommandLine(argc, argv, motorId);

  canOpenManager.initNode(motorId, hlcanopen::NodeManagerType::CLIENT);

  ros_info("starting manager thread");
  auto managerThread = thread([&](){
    canOpenManager.run();
  });
  ros_info("manager thread started");

  canOpenManager.writeSdoRemote</>()

  ros_info("stopping manager thread");
  canOpenManager.stop();
  managerThread.join();
  ros_info("done");
}

