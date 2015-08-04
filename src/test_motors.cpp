#include <iostream>
#include <string>

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"
#include "diana_powertrain/command_line.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/raii/scope_exit.h>

#include <boost/program_options.hpp>
#include <boost/core/ignore_unused.hpp>

INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;
using namespace std;
using namespace Td;

bool parseCommandLine(int argc, char** argv,
                      vector<int>& motorIds,
                      bool& doWizard
                     ) {
    using namespace boost::program_options;
    options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help messages")
    ("no-wizard,n", "skip the test wizard");
    prepareParseMotorIds(desc);
    variables_map varsMap;

    try {
        store(parse_command_line(argc, argv, desc), varsMap);

        if(varsMap.count("help")) {
            cout << desc << endl;
            return false;
        }

        if(varsMap.count("no-wizard")) {
            doWizard = false;
        } else {
            doWizard = true;
        }

        if(!parseMotorIds(varsMap, motorIds)) {
            return false;
        }

    } catch (error& e) {
        ros_error(toString("Unable to parse description: ",  e.what()));
        return false;
    }

    return true;
}

bool doWizard;

bool yesNoQuestion(const char* msg) {
    if(!doWizard) {
        return true;
    }
    std::cout << msg << "\n" << std::endl;

    return readYesNoCommandline();
}

int main(int argc, char** argv) {
    Pci7841Card card(0, 0);

    if(!card.open()) {
        ros_error("unable to open p7841 card");
        return -1;
    }

    hlcanopen::CanOpenManager<Pci7841Card> canOpenManager(card);
    canOpenManager.setupLogging();

    vector<int> motorIds;

    parseCommandLine(argc, argv, motorIds, doWizard);
    queue<int> motorsToTry;
    for(int id : motorIds) {
        motorsToTry.push(id);
    }

    ros_info("starting manager thread");
    auto managerThread = thread([&]() {
        canOpenManager.run();
    });
    ros_info("manager thread started");

    while(motorsToTry.size() > 0) {
        int motorId = motorsToTry.front();
        motorsToTry.pop();

        Motor<Pci7841Card> motor(canOpenManager, (unsigned int)motorId);
        scope_exit([&motor]() {
            motor.disable().wait();
        });

        ros_info(Td::toString("Starting test of motor: ", motorId));

        ros_info("setting profiled velocity mode");
        motor.setOperationMode(PROFILED_VELOCITY).wait();

        ros_info("enabling motor");
        motor.enable().wait();
        if(!yesNoQuestion("is the motor actived? it shoud not rotate freely")) {
            ros_error("unable to start motor.");
            std::cout << "if the motor vibrated, then it is possible that the signal or the phase wiring"
                      << "is not properly connected " << std::endl;
            return false;
        }

        ros_info("start motor");
        motor.setVelocity(10).wait();
        motor.start().wait();

        if(!yesNoQuestion("is the motor spinning?")) {
            ros_error("unable to make the motor move.");
            return false;
        }

        ros_info("the motor is ok");
    }

    ros_info("stopping manager thread");
    canOpenManager.stop();
    managerThread.join();
    ros_info("done");
}

