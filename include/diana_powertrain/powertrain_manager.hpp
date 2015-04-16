#ifndef POWERTRAIN_MANAGER_HPP
#define POWERTRAIN_MANAGER_HPP

#include <hlcanopen/can_open_manager.hpp>

template <class T> class PowertrainManager {
public:
  PowertrainManager(T card) : manager(card) {}
  ~PowertrainManager() {}

  void initiate_clients() {
    std::array<int, 4> clients_ids = {11, 12, 13, 14};
    std::for_each(clients_ids.begin(), clients_ids.end(), [&](int clientId) {
      manager.initNode(clientId, hlcanopen::NodeManagerType::CLIENT);
    });

    manager.run();

  }

  void run() {

  }

private:
  hlcanopen::CanOpenManager<T> manager;
};

#endif // POWERTRAIN_MANAGER_HPP
