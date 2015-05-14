#ifndef POWERTRAIN_MANAGER_HPP
#define POWERTRAIN_MANAGER_HPP

#include "diana_powertrain/motor.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include <cmath>

#include <hlcanopen/can_open_manager.hpp>

#include <vector>

#define WHEEL_SEPARATION 0.03

/* TODO: assign values */
enum {
  RIGHT_REAR_INDEX = 1,
  RIGHT_FRONT_INDEX = 2,
  LEFT_FRONT_INDEX = 3,
  LEFT_REAR_INDEX = 4
};

enum {
  RIGHT_REAR_ID = 11,
  RIGHT_FRONT_ID = 12,
  LEFT_FRONT_ID = 13,
  LEFT_REAR_ID = 14
};

template <class T> class PowertrainManager {
public:
  PowertrainManager(T& card) : manager(card, std::chrono::milliseconds(50)) {}

  PowertrainManager(const PowertrainManager<T>& oth) = delete;

  ~PowertrainManager() {
    set_motors_enabled(false);
    manager.stop();
    canOpenManagerThread.join();
  }

  void initiate_clients(const std::vector<int>& motorIds) {
    Td::ros_info("initiating clients");

    for(int motorId : motorIds) {
      manager.initNode(motorId, hlcanopen::NodeManagerType::CLIENT);
      motors.push_back(Motor<T>(manager, motorId));
    }

    Td::ros_info("starting CANopen manager thread");
    canOpenManagerThread = std::thread([&](){
      manager.run();
    });
  }

  void evaluate_velocities(double linear_v, double angular_v, float& right_v,
                           float& left_v) {
    right_v = (linear_v + angular_v * WHEEL_SEPARATION / 2.0);
    left_v = (linear_v - angular_v * WHEEL_SEPARATION / 2.0);
    Td::ros_info(Td::toString("evaluated velocity: [left wheels: ", left_v, "] right wheels: [", right_v, "]"));
  }

  void reset_motors() {
    Td::ros_info("Resetting motors");

    std::for_each(motors.begin(), motors.end(), [](Motor<T>& m) {
      Td::ros_info(Td::toString("Set command mode for id: ", m.getId()));
      bool ok = m.setCommandMode();
      if(ok) {
        Td::ros_info(Td::toString("Command mode set for motor ", m.getId()));
      } else {
        Td::ros_warn(Td::toString("Command mode NOT set for motor ", m.getId()));
      }
    });
  }

  void printMotorsStatusWord() {
    for(Motor<T>& m: motors) {
      int motorId = m.getId();
      StatusWord statusWord = m.getStatusWord().get().value;
      std::cout << "StatusWord of motor " << motorId << ": \n" << statusWord.toStringFull() << " \n";
    }
  }

  void setControlWord(ControlWordCommand command) {
    for(Motor<T>& m: motors) {
      int motorId = m.getId();
      Td::ros_info(Td::toString("Setting control word of motor: " + motorId));
      m.setControlWord(command);
    }
  }


  void printMotorsOperationMode() {
    for(Motor<T>& m: motors) {
      int motorId = m.getId();
      ModeOfOperation mode = m.getOperationMode().get().value;
      std::cout << "Operation mode of motor " << motorId << ": \n" << mode << " \n";
    }
  }

  void setMotorsOperationMode(ModeOfOperation mode) {
    std::for_each(motors.begin(), motors.end(), [mode](Motor<T>& m) {
      m.setOperationMode(mode);
    });
  }

  std::future<bool> set_motors_enabled(bool enabled) {
    Td::ros_info(Td::toString("Set motor enabled: ", enabled));
    std::vector<std::future<MotorAsyncResult>> results;

    for(Motor<T>& m: motors) {
      MotorAsyncResult r;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      if(enabled)  {
       r = m.enable().get();
      } else {
       r = m.disable().get();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      if(r.ok) {
        Td::ros_info(Td::toString("Motor ", m.getId(), enabled ? " enabled" : " disabled"));
      } else {
        Td::ros_error(Td::toString("Motor ", m.getId(), " NOT enabled"));
      }
    }
    return std::async(std::launch::deferred, [](){ return true; });
  }

  bool set_velocity(double linear_v, double angular_v) {
    float right_v, left_v;

    evaluate_velocities(linear_v, angular_v, right_v, left_v);

    const auto MIN_V = -1.f;
    const auto MAX_V = 1.f;

    if(right_v > MAX_V || right_v < MIN_V) {
      Td::ros_warn(Td::toString("velocity ", right_v, " is not supported by right wheels. clamping"));
      right_v = clamp(right_v, MIN_V, MAX_V );
    }
    if(left_v > MAX_V || left_v < MIN_V) {
      Td::ros_warn(Td::toString("velocity ", left_v, " is not supported by left wheels. clamping"));
      left_v = clamp(left_v, MIN_V, MAX_V );
    }

    std::vector<std::future<MotorAsyncResult>> results;


    for(Motor<T> motor : motors) {
      if(motor.getId() == 11 || motor.getId() == 14) {
        motor.setVelocity(right_v);
      } else {
        motor.setVelocity(left_v);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    for(std::future<MotorAsyncResult>& r: results) {
      const unsigned int  pause = 250;
      std::this_thread::sleep_for(std::chrono::milliseconds(pause));
      MotorAsyncResult result =  r.get();
      if(!result.ok)  {
        Td::ros_warn("Error while setting velocity of motor");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(pause));
    }

    return true;
  }

  std::vector<Motor<T>>& getMotors() {
    return motors;
  }

private:
  hlcanopen::CanOpenManager<T> manager;
  std::vector<Motor<T>> motors;
  std::thread canOpenManagerThread;
};

#endif // POWERTRAIN_MANAGER_HPP
