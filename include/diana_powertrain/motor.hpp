#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>

#include "diana_powertrain/consts.hpp"
#include "diana_powertrain/elmo_structs.hpp"

#include <hlcanopen/can_open_manager.hpp>
#include <hlcanopen/types.hpp>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/async/futures.h>
#include <team_diana_lib/enum/enum.h>

#include "utils.hpp"

#include <functional>


struct MotorAsyncResult {
  bool ok;
};

template <typename T> struct  MotorAsyncValue : public MotorAsyncResult {
  MotorAsyncValue(const T& value) : value(value) {
    ok = true;
  }

  T value;
};

template <typename T> MotorAsyncValue<T> make_motor_async_value(const T& value, bool ok = true) {
    MotorAsyncValue<T> v(value);
    v.ok = ok;
    return v;
}



std::future<MotorAsyncResult> ifResultOkThen(std::future<MotorAsyncResult> f, std::function<MotorAsyncResult()> fun) {
  return Td::then(std::move(f), [fun](MotorAsyncResult r) {
    Td::ros_info(Td::toString(" last result is ", r.ok));
    if(r.ok == true) {
      return fun();
    } else {
      MotorAsyncResult failedRes;
      failedRes.ok = false;
      return failedRes;
    }
  });
}

template <class T> class Motor {

public:
  Motor(hlcanopen::CanOpenManager<T>& canOpenManager, hlcanopen::NodeId id) :
    manager(canOpenManager),
    nodeId(id) {

  }

  Motor(const Motor<T>& oth) :
    manager(oth.manager),
    nodeId(oth.nodeId) {

  }

  ~Motor() {}

  std::future<MotorAsyncResult> enable() {
    return send_msg_async("MO=1", "enable()");
  }
  std::future<MotorAsyncResult> disable() {
    return send_msg_async("MO=0", "disable()");
  }

  std::future<MotorAsyncResult> start() {
    return send_msg_async("BG", "start()");
  }

  std::future<MotorAsyncResult> stop() {
    return send_msg_async("ST", "stop()");
  }

  template <typename K> std::future<MotorAsyncResult> completeWriteResult(
    std::future<hlcanopen::SdoResponse<K>>&& writeResult, std::string desc = "write" ) {
    return Td::then(std::move(writeResult), [desc](hlcanopen::SdoResponse<bool> writeResult) {
      bool ok = writeResult.get();
      MotorAsyncResult asyncRes;
      asyncRes.ok = ok;
      if(!ok) {
        Td::ros_warn(desc + " failed");
      }
      return asyncRes;

    });
  }

  std::future<MotorAsyncResult> setControlWord(ControlWordCommand command) {
    uint32_t controlWord = 0;
    controlWord |= getControlWordCommandBits(command);
    auto res = manager.template writeSdoRemote(nodeId, CONTROL_WORD, controlWord, 2000);
    return completeWriteResult(std::move(res));
  }

  std::future<MotorAsyncValue<StatusWord>> getStatusWord() {
    Td::ros_info("requestig status word");
    auto res = manager.template readSdoRemote<uint32_t>(nodeId, STATUS_WORD, 1000);
    return Td::then(std::move(res), [](hlcanopen::SdoResponse<uint32_t> readResult) {
      if(readResult.ok()) {
        uint32_t statusWordValue = readResult.get();
        StatusWord statusWord(statusWordValue);
        return make_motor_async_value(statusWord);
      } else {
        Td::ros_warn("Unable to read status word");
        return make_motor_async_value(StatusWord(0), false);
      }
    });
  }

  std::future<MotorAsyncResult> setOperationMode(ModeOfOperation mode) {
    uint32_t value = Td::to_int(mode);
    auto res = manager.template writeSdoRemote(nodeId, MODE_OF_OPERATION, value, 1000);
    return completeWriteResult(std::move(res), "operation mode write");
  }

  std::future<MotorAsyncValue<ModeOfOperation>> getOperationMode() {
    Td::ros_info("requestig mode of operation");
    auto res = manager.template readSdoRemote<uint32_t>(nodeId, MODE_OF_OPERATION_DISPLAY, 1000);
    return Td::then(std::move(res), [](hlcanopen::SdoResponse<uint32_t> readResult) {
      if(readResult.ok()) {
        ModeOfOperation modeOfOperation = (ModeOfOperation) readResult.get();
        return make_motor_async_value(modeOfOperation);
      } else {
        Td::ros_warn("Unable to read operation mode");
        return make_motor_async_value(ModeOfOperation::NO_MODE, false);
      }
    });
  }

//   std::future<MotorAsyncResult> setCommandMode() {
  bool setCommandMode() {
    manager.startRemoteNode(nodeId);
    mssleep(4000);
    return manager.template writeSdoRemote<uint32_t>(nodeId, OS_COMMAND_MODE, 0).get().get();
  }

  void send_msg_sync(const std::string& msg, const std::string& desc) {
    Td::ros_info(Td::toString("Sending msg to shell ", nodeId, ": ", msg));
    auto response = manager.writeSdoRemote(nodeId, writeIndex, msg);
    if(response.get().get() == false) {
      Td::ros_info(desc +" failed");
    }
  }

  std::future<MotorAsyncResult> send_msg_async(const std::string& msg, const std::string& desc) {
    Td::ros_info(Td::toString("Sending msg to shell ", nodeId, ": ", msg));
    auto res =  manager.writeSdoRemote(nodeId, OS_COMMAND_PROMPT_WRITE, msg, 1500);
    return Td::then(std::move(res), [desc](hlcanopen::SdoResponse<bool> writeResult) {
      bool ok = writeResult.get();
      MotorAsyncResult asyncRes;
      asyncRes.ok = ok;
      if(!ok) {
        Td::ros_warn(desc + " failed");
      }
      return asyncRes;

    });
  }

  std::future<MotorAsyncResult> setVelocity(float MetersPerSecond) {
//     return setJVVelocity(MetersPerSecond*MPS_JV_FACTOR);

    // TODO: this is still in testing:
    return setVelocitySdoOnly(MetersPerSecond*MPS_JV_FACTOR);
  }

  std::future<MotorAsyncResult> setVelocitySdoOnly(int value) {
    auto res = manager.template writeSdoRemote(nodeId, TARGET_VELOCITY, value, 1500);
    return completeWriteResult(std::move(res));
  }


  std::future<MotorAsyncValue<float>> getVelocity() {
    auto res = manager.template readSdoRemote<int32_t>(nodeId, VELOCITY_ACTUAL_VALUE, 1500);
    return Td::then(std::move(res), [](hlcanopen::SdoResponse<int32_t> velocityResult) {
      bool ok = velocityResult.ok();
      float vel = velocityResult.get();
      return make_motor_async_value(vel, ok);
    });
  }

  int getId() const {
    return nodeId;
  }

private:

  int clampJVToSafe(int speed) {
    bool clamped = false;

    if(speed > SPEED_JV_LIMIT) {
      return SPEED_JV_LIMIT;
      clamped = true;
    } else if(speed < -SPEED_JV_LIMIT) {
      return -SPEED_JV_LIMIT;
      clamped = true;
    }

    if(clamped) {
      Td::ros_warn(Td::toString("The requested speed was clamped to: ", speed ,"JV in order to maintain safety margins"));
    }

    return speed;
  }

  std::future<MotorAsyncResult> setJVVelocity(int velocity) {
    velocity = clampJVToSafe(velocity);

    std::future<MotorAsyncResult> res;
    res = send_msg_async(Td::toString("JV=", velocity), Td::toString("JV=", velocity));

    return ifResultOkThen(std::move(res), [&, velocity]() {
      if(velocity == 0) {
        return stop().get();
      } else {
        return start().get();
      }
    });
  }

  int getJVVelocity() {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "JV");
    if(response.get().get() == false) {
      Td::ros_error("getSpeed() failed: unable to fetch value");
      return -1; // Return error properly
    }

    while(true) {
      auto response = manager.template readSdoRemote<uint8_t>(nodeId, OS_COMMAND_PROMPT_STATUS);
      if(response.get().get() == 0x1)
        break;
      else if (response.get().get() == 0x3) {
        Td::ros_error("getSpeed() failed: command rejected");
        return -1; // Return error properly
      }
      else if (response.get().get() != 0xFF) {
        Td::ros_error("getSpeed() failed: unexpected value in 0x1023.2");
        return -1; // Return error properly
      }
      mssleep(500); // 500 ms
    }

    auto result = manager.template readSdoRemote<std::string>(nodeId, OS_COMMAND_PROMPT_READ);
    return std::stoi(result.get().get());
  }


private:
  hlcanopen::CanOpenManager<T>& manager;
  hlcanopen::NodeId nodeId;

  static hlcanopen::SDOIndex writeIndex;
  static hlcanopen::SDOIndex statusIndex;
  static hlcanopen::SDOIndex readIndex;

};


#endif // MOTOR_HPP
