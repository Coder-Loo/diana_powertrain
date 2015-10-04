#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>

#include "diana_powertrain/consts.hpp"
#include "diana_powertrain/shell.hpp"
#include "diana_powertrain/elmo_structs.hpp"

#include <hlcanopen/can_open_manager.hpp>
#include <hlcanopen/types.hpp>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/enum/enum.h>

#include <boost/lexical_cast.hpp>

#include <folly/futures/Future.h>

#include "utils.hpp"

#include <functional>

class Motor {

public:
    Motor(hlcanopen::CanOpenManager& canOpenManager, hlcanopen::NodeId id);

    Motor(const Motor& oth);

    ~Motor();

    folly::Future<folly::Unit> enable();

    folly::Future<folly::Unit> disable();

    folly::Future<folly::Unit> start();

    folly::Future<folly::Unit> stop();

    folly::Future<folly::Unit> setControlWord(ControlWordCommand command);

    folly::Future<StatusWord> getStatusWord();

    folly::Future<folly::Unit> setOperationMode(ModeOfOperation mode);

    folly::Future<ModeOfOperation> getOperationMode();

    bool setCommandMode();

    void send_msg_sync(const std::string& msg, const std::string& desc);

    folly::Future<folly::Unit> send_msg_async(const std::string& msg);

    folly::Future<float> getVelocity();

    folly::Future<folly::Unit> setVelocity(float revolutionsPerSecond);

    folly::Future<folly::Unit> setVelocitySdoOnly(int jvValue);

    void setVelocityPdo(float revolutionsPerSecond);

    float getVelocityPdo();

    hlcanopen::NodeId getId() const;

    folly::Future<uint8_t> getErrorRegister();

    folly::Future<std::string> getManufacturerDeviceName();

    folly::Future<std::string> getManufacturerHardwareVersion();

    folly::Future<std::string> getManufacturerSoftwareVersion();

    // parameters methods ////////////////////////////////////
    // these methods are only useful for dumping informations
    folly::Future<float> getContinousCurrentLimit();

    folly::Future<float> getPeakCurrentLimit();

    folly::Future<int> getPeakCurrentDuration();

    folly::Future<float> getElmoTemperature();

    folly::Future<int> getFeedbackCountsPerRevolution();

    folly::Future<int> getMotorPolePairs();

    folly::Future<std::string> getOSCommandResult(const std::string& command);

private:

    int clampJVToSafe(int speed);

    folly::Future<folly::Unit> setJVVelocity(int velocity);

    int getJVVelocity();


private:
    hlcanopen::CanOpenManager& manager;
    hlcanopen::NodeId nodeId;

};


#endif // MOTOR_HPP
