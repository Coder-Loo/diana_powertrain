#ifndef POWERTRAIN_MANAGER_HPP
#define POWERTRAIN_MANAGER_HPP

#include <hlcanopen/can_open_manager.hpp>

#define WHEEL_SEPARATION 0.03

/* TODO: assign values */
#define RIGHT_FRONT
#define RIGHT_REAR
#define LEFT_FRONT
#define LEFT_REAR

#define ID_RIGHT_FRONT
#define ID_RIGHT_REAR
#define ID_LEFT_FRONT
#define ID_LEFT_REAR

template <class T> class PowertrainManager {
public:
  PowertrainManager(T card) : manager(card) {}
  ~PowertrainManager() {}

  void initiate_clients() {
    std::array<int, 4> clients_ids = {11, 12, 13, 14};
    std::for_each(clients_ids.begin(), clients_ids.end(), [&](int clientId) {
      manager.initNode(clientId, hlcanopen::NodeManagerType::CLIENT);
    });
    for(auto i = 0; i < 4; i++) {
        motors[i] = Motor(manager, client_ids[i]);
    }

    manager.run();

  }

  void run() {

  }

  void evaluate_velocities(double linear_v, double angular_v, double& right_v,
                           double& left_v) {
    right_v = (linear_v + angular_v * WHEEL_SEPARATION / 2.0);
    left_v = (linear_v - angular_v * WHEEL_SEPARATION / 2.0);
  }

  std::future<bool> set_velocity(double linear_v, double angular_v) {
    double right_v, left_v; /* XXX: what type should be the velocities? */
    evaluate_velocities(linear_v, angular_v, right_v, left_v);

    motors[RIGHT_FRONT].setSpeed(right_v);
    motors[RIGHT_REAR].setSpeed(right_v);
    motors[LEFT_FRONT].setSpeed(left_v);
    motors[LEFT_REAR].setSpeed(left_v);

    return ;
  }

private:
  hlcanopen::CanOpenManager<T> manager;
  std::array<hlcanopen::Motor, 4> motors;
};

#endif // POWERTRAIN_MANAGER_HPP
