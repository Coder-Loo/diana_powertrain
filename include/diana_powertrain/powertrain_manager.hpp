#ifndef POWERTRAIN_MANAGER_HPP
#define POWERTRAIN_MANAGER_HPP

#include <hlcanopen/can_open_manager.hpp>

template <class T> class PowertrainManager {
public:
  PowertrainManager(T card) : manager(card) {}
  ~PowertrainManager() {}

private:
  hlcanopen::CanOpenManager<T> manager;
};

#endif // POWERTRAIN_MANAGER_HPP
