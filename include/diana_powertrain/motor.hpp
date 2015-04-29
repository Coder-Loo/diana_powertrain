#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>

#include <hlcanopen/can_open_manager.hpp>
#include <hlcanopen/types.hpp>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include "utils.hpp"


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

  void enable() {
    send_msg_sync("MO=1", "enable()");
  }
  void disable() {
    send_msg_sync("MO=0", "disable()");
  }

  void start() {
    send_msg_sync("BG", "start()");
  }

  void stop() {
    send_msg_sync("ST", "stop()");
  }

  void send_msg_sync(const std::string& msg, const std::string& desc) {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, msg);
    if(response.get().get() == false) {
      Td::ros_info(desc +" failed");
    }
  }

  void setSpeed(int speed) {
    if(speed > 0) {
      start();
    } else {
      disable();
    }
    auto response = manager.writeSdoRemote(nodeId, writeIndex, Td::toString("JV=", speed));
    if(response.get().get() == false) {
      Td::ros_info("setSpeed() failed");
    }
  }

  int getSpeed() {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "JV");
    if(response.get().get() == false) {
      Td::ros_error("getSpeed() failed: unable to fetch value");
      return -1; // Return error properly
    }

    while(true) {
      auto response = manager.template readSdoRemote<uint8_t>(nodeId, statusIndex);
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

    auto result = manager.template readSdoRemote<std::string>(nodeId, readIndex);
    return std::stoi(result.get().get());
  }

private:
  hlcanopen::CanOpenManager<T>& manager;
  hlcanopen::NodeId nodeId;

  static hlcanopen::SDOIndex writeIndex;
  static hlcanopen::SDOIndex statusIndex;
  static hlcanopen::SDOIndex readIndex;

};

template<class T> hlcanopen::SDOIndex Motor<T>::writeIndex = hlcanopen::SDOIndex(0x1023, 1);
template<class T> hlcanopen::SDOIndex Motor<T>::statusIndex = hlcanopen::SDOIndex(0x1023, 2);
template<class T> hlcanopen::SDOIndex Motor<T>::readIndex = hlcanopen::SDOIndex(0x1023, 3);

#endif // MOTOR_HPP
