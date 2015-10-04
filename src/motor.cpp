#include "diana_powertrain/motor.hpp"

Motor::Motor(hlcanopen::CanOpenManager& canOpenManager, hlcanopen::NodeId id) :
    manager(canOpenManager),
    nodeId(id) {

}

Motor::Motor(const Motor& oth) :
    manager(oth.manager),
    nodeId(oth.nodeId) {
}

Motor::~Motor() {}

folly::Future<folly::Unit> Motor::enable() {
    return send_msg_async("MO=1");
}
folly::Future<folly::Unit> Motor::disable() {
    return send_msg_async("MO=0");
}

folly::Future<folly::Unit> Motor::start() {
    return send_msg_async("BG");
}

folly::Future<folly::Unit> Motor::stop() {
    return send_msg_async("ST");
}

folly::Future<folly::Unit> Motor::setControlWord(ControlWordCommand command) {
    uint32_t controlWord = 0;
    controlWord |= getControlWordCommandBits(command);
    return manager.template writeSdoRemote(nodeId, CONTROL_WORD, controlWord, 2000);
}

folly::Future<StatusWord> Motor::getStatusWord() {
    Td::ros_info("requestig status word");
    folly::Future<uint32_t> res = manager.template readSdoRemote<uint32_t>(nodeId, STATUS_WORD, 1000);
    return res.then([](uint32_t statusWordValue) {
        StatusWord statusWord(statusWordValue);
        return statusWord;
    });
}

folly::Future<folly::Unit> Motor::setOperationMode(ModeOfOperation mode) {
    uint32_t value = Td::to_int(mode);
    return manager.template writeSdoRemote(nodeId, MODE_OF_OPERATION, value, 1000);
}

folly::Future<ModeOfOperation> Motor::getOperationMode() {
    Td::ros_info("requestig mode of operation");
    folly::Future<uint32_t> res = manager.template readSdoRemote<uint32_t>(nodeId, MODE_OF_OPERATION_DISPLAY, 1000);
    return res.then([](uint32_t modeOfOperationValue) {
        ModeOfOperation modeOfOperation = (ModeOfOperation) modeOfOperationValue;
        return modeOfOperation;
    });
}

//   std::future<MotorAsyncResult> setCommandMode() {
bool Motor::setCommandMode() {
//         manager.startRemoteNode(nodeId);
//         mssleep(4000);
    return manager.template writeSdoRemote<uint32_t>(nodeId, OS_COMMAND_MODE, 0).wait().hasValue();
}

void Motor::send_msg_sync(const std::string& msg, const std::string& desc) {
    Td::ros_info(Td::toString("Sending msg to shell ", nodeId, ": ", msg));
    auto response = manager.writeSdoRemote(nodeId, OS_COMMAND_PROMPT_WRITE, msg);
    response.wait();
    if(response.hasException()) {
        Td::ros_info(desc +" failed");
    }
}

folly::Future<folly::Unit> Motor::send_msg_async(const std::string& msg) {
    Td::ros_info(Td::toString("Sending msg to shell ", nodeId, ": ", msg));
    return manager.writeSdoRemote(nodeId, OS_COMMAND_PROMPT_WRITE, msg, 1500);
}

folly::Future<folly::Unit> Motor::setVelocity(float revolutionsPerSecond) {
    int jvValue = revolutionsPerSecond * JV_RPS_FACTOR;
    return setVelocitySdoOnly(jvValue);
}

folly::Future<folly::Unit> Motor::setVelocitySdoOnly(int jvValue) {
    jvValue = clampJVToSafe(jvValue);
    Td::ros_info(Td::toString("vel of motor ", nodeId,  " is ", jvValue, "JV"));
    return manager.template writeSdoRemote(nodeId, TARGET_VELOCITY, jvValue, 1500);
}

void Motor::setVelocityPdo(float revolutionsPerSecond) {
    int jvValue = revolutionsPerSecond * JV_RPS_FACTOR;
    jvValue = clampJVToSafe(jvValue);
    manager.template writeSdoLocal(nodeId, TARGET_VELOCITY, jvValue);
//     manager.writeRPDO(nodeId, TARGET_VELOCITY_COB_ID);
}

// Get velocity in revolutions per second.
folly::Future<float> Motor::getVelocity() {
    auto res = manager.template readSdoRemote<int32_t>(nodeId, VELOCITY_SENSOR_ACTUAL_VALUE, 1500);
    return res
    .then([&](int32_t velValue) {
        Td::ros_info(Td::toString("JV OF MOTOR ", nodeId, " IS ", velValue , "JV"));
        return (float)((float)velValue/JV_RPS_FACTOR);
    });
//         .onError([]() {
//             Td::ros_error("unable to get velocity");
//             return (float)0.0f;
//         })
}

float Motor::getVelocityPdo() {
//     return manager.readSdoLocal<float>(nodeId, VELOCITY_ACTUAL_VALUE);
  throw new std::runtime_error("not implemented yet");
  return 0;
}

hlcanopen::NodeId Motor::getId() const {
    return nodeId;
}

folly::Future<uint8_t> Motor::getErrorRegister() {
    return manager.template readSdoRemote<uint8_t>(nodeId, ERROR_REGISTER, 1500);
}

folly::Future<std::string> Motor::getManufacturerDeviceName() {
    return manager.template readSdoRemote<std::string>(nodeId, MANUFACTURER_DEVICE_NAME, 1500);
}

folly::Future<std::string> Motor::getManufacturerHardwareVersion() {
    return manager.template readSdoRemote<std::string>(nodeId, MANUFACTURER_HARDWARE_VERSION, 1500);
}

folly::Future<std::string> Motor::getManufacturerSoftwareVersion() {
    return manager.template readSdoRemote<std::string>(nodeId, MANUFACTURER_SOFTWARE_VERSION, 1500);
}

// parameters methods ////////////////////////////////////
// these methods are only useful for dumping informations
folly::Future<float> Motor::getContinousCurrentLimit() {
    return getOSCommandResult("CL[1]").then([](std::string v) {
        return boost::lexical_cast<float>(v);
    });
}

folly::Future<float> Motor::getPeakCurrentLimit() {
    return getOSCommandResult("PL[1]").then([](std::string v) {
        return boost::lexical_cast<float>(v);
    });
}

folly::Future<int> Motor::getPeakCurrentDuration() {
    return getOSCommandResult("PL[2]").then([](std::string v) {
        return (int)boost::lexical_cast<float>(v);
    });
}

folly::Future<float> Motor::getElmoTemperature() {
    return getOSCommandResult("TI[1]").then([](std::string v) {
        return boost::lexical_cast<float>(v);
    });
}

folly::Future<int> Motor::getFeedbackCountsPerRevolution() {
    return getOSCommandResult("CA[18]").then([](std::string v) {
        return boost::lexical_cast<int>(v);
    });
}

folly::Future<int> Motor::getMotorPolePairs() {
    return getOSCommandResult("CA[19]").then([](std::string v) {
        return boost::lexical_cast<int>(v);
    });
}
// parameters method end /////////////////////////////////////////

//     folly::Future<std::string> getOSCommandResult(const std::string& command) {
//         std::cout << "get os command result start" << std::endl;
//         std::function<folly::Future<std::string>(uint8_t)> waitForCompletionLambda =
//           [waitForCompletionLambda, command, this](uint8_t status) {
//             folly::Promise<std::string> prom;
//             std::cout << "1" << std::endl;
//             auto fut = prom.getFuture();
//             switch(status) {
//             case COMPLETED_NO_REPLY: {
//                 std::cout << "2" << std::endl;
//                 prom.setValue("");
//                 Td::ros_info("got completed without reply");
//                 return fut;
//             }
//             case COMPLETED_REPLY: {
//                 std::cout << "3" << std::endl;
//                 Td::ros_info("got completed reply");
//                 auto res = manager.template readSdoRemote<std::string>(nodeId, OS_COMMAND_PROMPT_READ);
//                 return res;
//             }
//             case REJECTED_REPLY: {
//                 std::cout << "4" << std::endl;
//                 Td::ros_error("got command rejected");
//                 prom.setValue("");
//                 return fut;
//             }
//             case EXECUTING: {
//                 std::cout << "5" << std::endl;
//                 Td::ros_info("got executing...");
//                 uint8_t sta = manager.template readSdoRemote<std::uint8_t>(nodeId, OS_COMMAND_PROMPT_STATUS).wait().value();
//                 return waitForCompletionLambda(sta);
//                 auto newStatusFuture = manager.template readSdoRemote<std::uint8_t>(nodeId, OS_COMMAND_PROMPT_STATUS);
//                 return newStatusFuture.then(waitForCompletionLambda);
//             }
//             default: {
//                 std::cout << "6" << std::endl;
//                 Td::ros_error("received unknown resut");
//                 prom.setValue("");
//                 return fut;
//             }
//             }
//             std::cout << "7" << std::endl;
//         };
//
//
//
//         folly::Future<folly::Unit> writeRes;
//         writeRes = manager.writeSdoRemote(nodeId, OS_COMMAND_PROMPT_WRITE, command);
//
//         return writeRes.onError([](const std::exception e) {
//             std::cout << "onError called" << std::endl;
//             Td::ros_error(Td::toString("unable to read OS command result: ", e.what()));
//         })
//         .then([=]() {
//             std::cout << "0" << std::endl;
//             return waitForCompletionLambda(EXECUTING);
//         });
//     }

// // This can be improved, see commented version
folly::Future<std::string> Motor::getOSCommandResult(const std::string& command) {
    auto res = manager.writeSdoRemote(nodeId, OS_COMMAND_PROMPT_WRITE, command);

    if(res.wait().hasValue()) {
        bool waitingForCompletion = true;
        while(waitingForCompletion) {
            auto res = manager.template readSdoRemote<uint8_t>(nodeId, OS_COMMAND_PROMPT_STATUS);
            if(!res.wait().hasValue()) {
                Td::ros_error("unable to get status");
                return "";
            }
            uint8_t status = res.get();
            switch(status) {
            case COMPLETED_NO_REPLY:
                waitingForCompletion = false;
                break;
            case COMPLETED_REPLY:
            case REJECTED_REPLY: {
                if(status == REJECTED_REPLY) {
                    Td::ros_error("command rejected");
                }
                auto res = manager.template readSdoRemote<std::string>(nodeId, OS_COMMAND_PROMPT_READ);
                if(res.wait().hasValue()) {
                    return res.value();
                } else {
                    Td::ros_error("unable to read response");
                    return "";
                }
            }
            waitingForCompletion = false;
            break;
            case EXECUTING:
                waitingForCompletion = true;
                break;
            default:
                throw std::runtime_error("unknown answer at sdoIndex (0x1023,2)");
                break;
            }
        }

    } else {
        Td::ros_error(Td::toString("unable to write OS command result"));
        return "";
    }

    return "";
}


int Motor::clampJVToSafe(int speed) {
    bool clamped = false;

    if(speed > SPEED_JV_LIMIT) {
        speed = SPEED_JV_LIMIT;
        clamped = true;
    } else if(speed < -SPEED_JV_LIMIT) {
        speed = -SPEED_JV_LIMIT;
        clamped = true;
    }

    if(clamped) {
        Td::ros_warn(Td::toString("The requested speed was clamped to: ", speed ,"JV in order to maintain safety margins"));
    }

    return speed;
}

folly::Future<folly::Unit> Motor::setJVVelocity(int velocity) {
    velocity = clampJVToSafe(velocity);

    folly::Future<folly::Unit> res;
    res = send_msg_async(Td::toString("JV=", velocity));

    return res.then([&, velocity]() {
        if(velocity == 0) {
            return stop().get();
        } else {
            return start().get();
        }
    });
}

int Motor::getJVVelocity() {
    auto response = manager.writeSdoRemote<std::string>(nodeId, OS_COMMAND_PROMPT_WRITE, "JV");
    response.wait();
    if(response.hasException() == false) {
        Td::ros_error("getSpeed() failed: unable to fetch value");
        return -1; // Return error properly
    }

    while(true) {
        auto response = manager.template readSdoRemote<uint8_t>(nodeId, OS_COMMAND_PROMPT_STATUS);
        response.wait();
        if(response.get() == 0x1)
            break;
        else if (response.get() == 0x3) {
            Td::ros_error("getSpeed() failed: command rejected");
            return -1; // Return error properly
        }
        else if (response.get() != 0xFF) {
            Td::ros_error("getSpeed() failed: unexpected value in 0x1023.2");
            return -1; // Return error properly
        }
    }

    auto result = manager.template readSdoRemote<std::string>(nodeId, OS_COMMAND_PROMPT_READ);
    std::string a;
    return std::stoi(result.get().c_str());
}
