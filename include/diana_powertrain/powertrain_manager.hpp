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
  PowertrainManager(T card) : manager(card) {}
  ~PowertrainManager() {}

  void initiate_clients() {
    std::array<int, 4> client_ids = {RIGHT_REAR_ID, RIGHT_FRONT_ID, LEFT_FRONT_ID, LEFT_REAR_ID};
    std::for_each(client_ids.begin(), client_ids.end(), [&](int clientId) {
      manager.initNode(clientId, hlcanopen::NodeManagerType::CLIENT);
    });
//     for(auto i = 0; i < 4; i++) {
//         motors[i] = Motor<T>(manager, client_ids[i]);
//     }
//     motors.push_back(Motor<T>(manager, client_ids[0]));
    motors.push_back(Motor<T>(manager, client_ids[1]));
//     motors.push_back(Motor<T>(manager, client_ids[2]));
//     motors.push_back(Motor<T>(manager, client_ids[3]));

    manager.run();
  }

  void run() {

  }

  void evaluate_velocities(double linear_v, double angular_v, double& right_v,
                           double& left_v) {
    right_v = (linear_v + angular_v * WHEEL_SEPARATION / 2.0);
    left_v = (linear_v - angular_v * WHEEL_SEPARATION / 2.0);
    Td::ros_info(Td::toString("evaluated velocity: [left wheels: ", left_v, "] right wheels: [", right_v, " ]"));
  }

  bool set_motors_enabled(bool enabled) {
    std::for_each(motors.begin(), motors.end(), [&](Motor<T>& m) {
      if(enabled) {
        m.enable();
      } else {
        m.disable();
      }
    });
  }

  bool set_velocity(double linear_v, double angular_v) {
    double right_v, left_v; /* XXX: what type should be the velocities? */
    evaluate_velocities(linear_v, angular_v, right_v, left_v);

    motors[0].setSpeed(right_v);
//     motors[RIGHT_FRONT_INDEX].setSpeed(right_v);
//     motors[RIGHT_REAR_INDEX].setSpeed(right_v);
//     motors[LEFT_FRONT_INDEX].setSpeed(left_v);
//     motors[LEFT_REAR_INDEX].setSpeed(left_v);

    return true;
  }

private:
  hlcanopen::CanOpenManager<T> manager;
  std::vector<Motor<T>> motors;
};

#endif // POWERTRAIN_MANAGER_HPP
