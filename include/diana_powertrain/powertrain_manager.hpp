#ifndef POWERTRAIN_MANAGER_HPP
#define POWERTRAIN_MANAGER_HPP

#include "diana_powertrain/motor.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

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

  void initiate_clients() {
    Td::ros_info("initiating clients");
    std::array<int, 4> client_ids = {RIGHT_REAR_ID, RIGHT_FRONT_ID, LEFT_FRONT_ID, LEFT_REAR_ID};
    std::for_each(client_ids.begin(), client_ids.end(), [&](int clientId) {
      manager.initNode(clientId, hlcanopen::NodeManagerType::CLIENT);
    });
//     for(auto i = 0; i < 4; i++) {
//         motors[i] = Motor<T>(manager, client_ids[i]);
//     }
//     motors.push_back(Motor<T>(manager, client_ids[0]));
    motors.push_back(Motor<T>(manager, client_ids[0]));
    motors.push_back(Motor<T>(manager, client_ids[1]));
    motors.push_back(Motor<T>(manager, client_ids[2]));
    motors.push_back(Motor<T>(manager, client_ids[3]));

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

//     std::transform(motors.begin(), motors.end(), results.begin(), [&](Motor<T>& m) {
//       std::future<MotorAsyncResult> r;
//       if(enabled) {
//         r = m.enable();
//       } else {
//         r = m.disable();
//       }
//       return r;
//     });
//
//     Td::ros_info(Td::toString("check results async: "));
//     return std::async(std::launch::deferred, [&]() {
//       return std::all_of(results.begin(), results.end(), [&](std::future<MotorAsyncResult>& motorOk) {
//         Td::ros_info(Td::toString("checking a motor: "));
//         return motorOk.get().ok == true;
//       });
//     });
    return std::async(std::launch::deferred, [](){ return true; });
  }

  bool set_velocity(double linear_v, double angular_v) {
    float right_v, left_v;
    evaluate_velocities(linear_v, angular_v, right_v, left_v);

    std::vector<std::future<MotorAsyncResult>> results;


    results.push_back(motors[0].setVelocity(right_v));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    results.push_back(motors[1].setVelocity(right_v));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    results.push_back(motors[2].setVelocity(left_v));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    results.push_back(motors[3].setVelocity(left_v));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
//     motors[RIGHT_FRONT_INDEX].setVelocity(right_v);
//     motors[RIGHT_REAR_INDEX].setVelocity(right_v);
//     motors[LEFT_FRONT_INDEX].setVelocity(left_v);
//     motors[LEFT_REAR_INDEX].setVelocity(left_v);

    for(std::future<MotorAsyncResult>& r: results) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      r.get();
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    return true;
  }

private:
  hlcanopen::CanOpenManager<T> manager;
  std::vector<Motor<T>> motors;
  std::thread canOpenManagerThread;
};

#endif // POWERTRAIN_MANAGER_HPP
