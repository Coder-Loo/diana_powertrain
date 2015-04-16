#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>

#include <hlcanopen/can_open_manager.hpp>
#include <hlcanopen/types.hpp>

#include "utils.hpp"


template <class T> class Motor {
  Motor(hlcanopen::CanOpenManager<T> canOpenManager, hlcanopen::NodeId id) :
    manager(canOpenManager),
    nodeId(id)
    {}
  ~Motor() {}

  void enable() {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "MO=1");
    if(response().get().get() == false)
      ros_info("enable() failed");
  }
  void disable() {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "MO=0");
    if(response().get().get() == false)
      ros_info("disable() failed");
  }
  void start() {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "BG");
    if(response().get().get() == false)
      ros_info("start() failed");
  }
  void setSpeed(int speed) {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "JV=" + speed);
    if(response().get().get() == false)
      ros_info("setSpeed() failed");
  }
  int getSpeed() {
    auto response = manager.writeSdoRemote(nodeId, writeIndex, "JV");
    if(response().get().get() == false) {
      ros_info("getSpeed() failed: unable to fetch value");
      return -1; // Return error properly
    }

    while(true) {
      auto response = manager.readSdoRemote<uint8_t>(nodeId, statusIndex);
      if(response.get().get() == 0x1)
	break;
      else if (response.get().get() == 0x3) {
	ros_info("getSpeed() failed: command rejected");
	return -1; // Return error properly
      }
      else if (response.get().get() != 0xFF) {
	ros_info("getSpeed() failed: unexpected value in 0x1023.2");
	return -1; // Return error properly
      }
      mssleep(500); // 500 ms
    }

    auto result = manager.readSdoRemote<std::string>(nodeId, readIndex);
    return std::stoi(result.get().get());
  }

private:
  hlcanopen::CanOpenManager<T> manager;
  hlcanopen::SDOIndex writeIndex(0x1023, 1);
  hlcanopen::SDOIndex statusIndex(0x1023, 2);
  hlcanopen::SDOIndex readIndex(0x1023, 3);
  hlcanopen::NodeId nodeId;

};


#endif // MOTOR_HPP
