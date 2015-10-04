#include <iostream>
#include <string>

#include "hlcanopen/logging/easylogging++.h"

#include "diana_powertrain/pci7841_card.hpp"
#include "diana_powertrain/command_line.hpp"
#include "diana_powertrain/consts.hpp"
#include "diana_powertrain/motor.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/raii/scope_exit.h>

#include <boost/program_options.hpp>
#include <boost/core/ignore_unused.hpp>

#include <thread>

INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;
using namespace std;
using namespace Td;

bool parseCommandLine(int argc, char** argv,
                      vector<int>& motorIds,
                      bool& doWizard,
                      bool& runAll,
                      int& runAllDuration,
                      float& velocityRotationPerSecond
                     ) {
    using namespace boost::program_options;
    options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help messages")
    ("no-wizard,n", "skip the test wizard")
    ("all,a", value<int>()->default_value(10)->implicit_value(true),
     "run all the motor at the same time, for the specified duration in seconds")
    ("speed,s", value<float>()->default_value(1),
     "use the motor specified velocity (in rotation per second)");
    prepareParseMotorIds(desc, false);
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

        if(varsMap.count("all") && !varsMap["all"].defaulted()) {
            runAll = true;
            runAllDuration = varsMap["all"].as<int>();
        } else {
            runAll = false;
        }

        velocityRotationPerSecond = varsMap["speed"].as<float>();

        if(!parseMotorIds(varsMap, motorIds, false)) {
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
    cout << msg << " [y/n] "<< "\n" << endl;

    return readYesNoCommandline();
}

int main(int argc, char** argv) {
    Pci7841Card card(0, 0);

    hlcanopen::CanOpenManager canOpenManager(card);
    canOpenManager.setupLogging();

    if(!card.open()) {
        ros_error("unable to open p7841 card");
        return -1;
    }

    vector<int> motorIds;
    bool runAll;
    int runAllDuration;
    float velocityRotationPerSecond;

    if(!parseCommandLine(argc, argv, motorIds, doWizard, runAll, runAllDuration, velocityRotationPerSecond)) {
        return -1;
    };


    cout << "starting manager thread" << endl;
    auto managerThread = thread([&]() {
        canOpenManager.run();
    });
    cout << "manager thread started" << endl;

    scope_exit stopThreadAtExit = scope_exit([&]() {
        cout << "stopping manager thread" << endl;
        canOpenManager.stop();
        managerThread.join();
    });

    if(runAll) {
        std::array<Motor, 4> motors {
            Motor(canOpenManager, 11),
            Motor(canOpenManager, 12),
            Motor(canOpenManager, 13),
            Motor(canOpenManager, 14)
        };
        scope_exit disableMotorAtExit = scope_exit([&motors]() {
            cout << "disabling motors" << endl;
            for(auto& motor : motors) {
                motor.disable().wait();
            }
        });

        for(auto& motor : motors) {
            motor.setCommandMode();
            motor.setControlWord(ControlWordCommand::ENABLE_OPERATION).wait();
            motor.setOperationMode(PROFILED_VELOCITY).wait();
            motor.enable().wait();
        }

        for(auto& motor : motors) {
            motor.setVelocity(velocityRotationPerSecond).wait();
        }

        cout << "Running motors for " << runAllDuration << " seconds " << endl;
        this_thread::sleep_for(chrono::seconds(runAllDuration));

    } else {

        queue<int> motorsToTry;
        for(int id : motorIds) {
            motorsToTry.push(id);
        }

        while(motorsToTry.size() > 0) {
            int motorId = motorsToTry.front();
            motorsToTry.pop();

            Motor motor(canOpenManager, (unsigned int)motorId);
            scope_exit disableMotorAtExit = scope_exit([&motor]() {
                cout << "disabling motor" << endl;
                motor.disable().wait();
            });

            cout << "Starting test of motor: " << endl;

            motor.setCommandMode();
            ros_info("setting profiled velocity mode");
            motor.setControlWord(ControlWordCommand::ENABLE_OPERATION).wait();
            motor.setOperationMode(PROFILED_VELOCITY).wait();

            ros_info("enabling motor");
            motor.enable().wait();
            if(!yesNoQuestion("is the motor actived? it shoud not rotate freely")) {
                ros_error("unable to start motor.");
                cout << "if the motor vibrated, then it is possible that the signal or the phase wiring"
                     << "is not properly connected " << endl;
                return false;
            }

            ros_info("start motor");
            motor.setVelocity(velocityRotationPerSecond).wait();
            motor.start().wait();

            if(!yesNoQuestion("is the motor spinning?")) {
                ros_error("unable to make the motor move.");
                return false;
            }
            if(!doWizard) {
                this_thread::sleep_for(chrono::milliseconds(500));
            }

            ros_info("the motor is ok");
        }
    }

    ros_info("done");
}

