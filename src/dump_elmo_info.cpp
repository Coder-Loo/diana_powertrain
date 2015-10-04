#include <iostream>
#include <string>
#include <iomanip>

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
                      vector<int>& motorIds) {
    using namespace boost::program_options;
    options_description desc("Options");
    desc.add_options()
    ("help,h", "Print help messages");
    prepareParseMotorIds(desc);
    variables_map varsMap;

    try {
        store(parse_command_line(argc, argv, desc), varsMap);

        if(varsMap.count("help")) {
            cout << desc << endl;
            return false;
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

int main(int argc, char** argv) {
    Pci7841Card card(0, 0);

    hlcanopen::CanOpenManager canOpenManager(card);
    auto executor = std::make_shared<hlcanopen::UniqueThreadExecutor>();
    canOpenManager.setupLogging();
    canOpenManager.setDefaultFutureExecutor(executor);

    if(!card.open()) {
        ros_error("unable to open p7841 card");
        return -1;
    }

    vector<int> motorIds;

    if(!parseCommandLine(argc, argv, motorIds)) {
      return -1;
    };

    queue<int> motorsToDump;
    for(int id : motorIds) {
        motorsToDump.push(id);
    }

    cout << "starting manager thread" << endl;
    auto managerThread = thread([&]() {
        canOpenManager.run();
    });
    cout << "manager thread started" << endl;

    scope_exit stopThreadAtExit = scope_exit([&](){
      cout << "stopping manager thread" << endl;
      canOpenManager.stop();
      managerThread.join();
    });

    while(motorsToDump.size() > 0) {
        int motorId = motorsToDump.front();
        motorsToDump.pop();

        Motor motor(canOpenManager, (unsigned int)motorId);

        cout << "\nInfo of elmo: " << motorId << endl;

        string deviceName = motor.getManufacturerDeviceName().get();
        string hardwareRevision = motor.getManufacturerHardwareVersion().get();
        string softwareVersion = motor.getManufacturerSoftwareVersion().get();
        float temperature = motor.getElmoTemperature().get();
        float continousCurrentLimit = motor.getContinousCurrentLimit().get();
        float peakCurrentLimit = motor.getPeakCurrentLimit().get();
        int peakCurrentDuration = motor.getPeakCurrentDuration().get();
        int countPerRevolution = motor.getFeedbackCountsPerRevolution().get();
        int numberOfPolePairs =  motor.getMotorPolePairs().get();
        ModeOfOperation modeOfOperation = motor.getOperationMode().get();
        StatusWord statusWord = motor.getStatusWord().get();

        const int w = 40;

        cout << endl;
        cout << setw(w) << std::left << "\tDevice Name:" << deviceName << endl;
        cout << setw(w) << std::left << "\tHardware revision:" << hardwareRevision << endl;
        cout << setw(w) << std::left << "\tSoftware version:" << softwareVersion << endl;
        cout << setw(w) << std::left << "\tCurrent Temperature:" << temperature << "C°" << endl;
        cout << setw(w) << std::left << "\t---" << endl;
        cout << setw(w) << std::left << "\tContinous current limit:"<< continousCurrentLimit << endl;
        cout << setw(w) << std::left << "\tPeak current limit:" << peakCurrentLimit << endl;
        cout << setw(w) << std::left << "\tPeak current duration:" << peakCurrentDuration << "s" << endl;
        cout << setw(w) << std::left << "\t---" << endl;
        cout << setw(w) << std::left << "\tFeedback count per revolution:" << countPerRevolution << endl;
        cout << setw(w) << std::left << "\tNumber of motor pole pairs:" << numberOfPolePairs << endl;
        cout << setw(w) << std::left << "\tMode of operation:" << modeOfOperation << endl;
        cout << setw(w) << std::left << "\tStatus:" << statusWord.getStatus() << endl;

        cout << endl;
    }

    cout << "done" << endl;
}

