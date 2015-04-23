#include <iostream>
#include <string>

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"
#include "diana_powertrain/command_line.hpp"
#include "diana_powertrain/utils.hpp"
#include "diana_powertrain/consts.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>



INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;
using namespace std;
using namespace Td;

bool parseCommandLine(int argc, char** argv, int& motorId) {
  using namespace boost::program_options;
  options_description desc("Options");
  desc.add_options()
    ("help,h", "Print help messages");
  prepareParseMotorId(desc);
  variables_map varsMap;

  try {
      store(parse_command_line(argc, argv, desc), varsMap);

      if(varsMap.count("help")) {
        cout << desc << endl;
        return false;
      }

      if(!parseMotorId(varsMap, motorId)) {
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
  if(!parseCommandLine(argc, argv, motorId)) {
    return -1;
  }

  canOpenManager.initNode(motorId, hlcanopen::NodeManagerType::CLIENT);

  ros_info("starting manager thread");
  auto managerThread = thread([&](){
    canOpenManager.run();
  });
  ros_info("manager thread started");

  canOpenManager.startRemoteNode(motorId);
  mssleep(1500);
  auto res = canOpenManager.writeSdoRemote<uint32_t>(motorId, OS_COMMAND_MODE, 0);

  if(!res.get().ok()) {
    ros_error("Unable to set command mode");
    return -1;
  }

  mssleep(1000);

  while(true) {
    string s;
    cout << "-->";
    cin >> s;
    cout << endl;
    if(s == "exit" || s == "q") {
      break;
    }
    auto res = canOpenManager.writeSdoRemote(motorId, OS_COMMAND_MODE, s);
    if(res.get().ok()) {
      ros_info("sent new command");
    } else {
      ros_error("error while sending new command");
    }
  }

  ros_info("stopping manager thread");
  canOpenManager.stop();
  managerThread.join();
  ros_info("done");
}

