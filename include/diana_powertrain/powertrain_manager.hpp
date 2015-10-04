#ifndef POWERTRAIN_MANAGER_HPP
#define POWERTRAIN_MANAGER_HPP

#include "diana_powertrain/motor.hpp"

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/strings/bit_printer.h>


#include <hlcanopen/can_open_manager.hpp>
#include <hlcanopen/executor/unique_thread_executor.hpp>

#include <boost/timer.hpp>

#include <cmath>
#include <vector>

#define WHEEL_SEPARATION 0.03

#define METER_PER_SECOND_REVOLUTION 0.562345084992573

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

class PowertrainManager {
public:
  PowertrainManager(hlcanopen::CanCard& card);

  PowertrainManager(const PowertrainManager& oth) = delete;

  ~PowertrainManager();

  void initiate_clients(const std::vector<int>& motorIds);

  /**
   * evaluate the velocity for the left and right side wheels.
   * the velocity of a side is related to the velocity of the other side
   * by angular_v. When angular_v is 0, the velocities are the same (no turn)
   * When angular_v is 1, then we turn left and the right wheels must have
   * the opposite velocities of the left wheels.
   */
  void evaluate_velocities_m_s(double linear_v, double angular_v, float& right_v,
                           float& left_v);

  void reset_motors();

  void printMotorsStatusWord();

  void setControlWord(ControlWordCommand command);

  void printMotorsOperationMode();

  void setMotorsOperationMode(ModeOfOperation mode);

  bool set_motors_enabled(bool enabled);

  // Get velocity in meter per second.
  folly::Future<float> get_velocity(unsigned int motorIndex);

  bool set_velocity(double linear_v, double angular_v);

  std::vector<Motor>& getMotors();

private:
  hlcanopen::CanOpenManager manager;
  std::vector<Motor> motors;
  std::thread canOpenManagerThread;
  std::shared_ptr<hlcanopen::UniqueThreadExecutor> futureExecutor;

};

#endif // POWERTRAIN_MANAGER_HPP
