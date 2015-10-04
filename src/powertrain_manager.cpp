#include "diana_powertrain/powertrain_manager.hpp"

  PowertrainManager::PowertrainManager(hlcanopen::CanCard& card) : manager(card, std::chrono::milliseconds(50)) {
    manager.setupLogging();
    manager.setDefaultFutureExecutor(futureExecutor);
  }

  PowertrainManager::~PowertrainManager() {
    set_motors_enabled(false);
    manager.stop();
    canOpenManagerThread.join();
  }

  void PowertrainManager::initiate_clients(const std::vector<int>& motorIds) {
    Td::ros_info("initiating clients");

    for(int motorId : motorIds) {
      manager.initNode(motorId, hlcanopen::NodeManagerType::CLIENT);
      motors.push_back(Motor(manager, motorId));

      //hlcanopen::PdoConfiguration configTargetVel(hlcanopen::RPDO, 1);

      //hlcanopen::COBIdPdoEntry cobIdPdoTargetVel;
      //cobIdPdoTargetVel.setCobId(hlcanopen::COBId(motorId, TARGET_VELOCITY_COB_ID));
      //cobIdPdoTargetVel.enable29bitId(false);
      //cobIdPdoTargetVel.enableRtr(false);
      //cobIdPdoTargetVel.enablePdo(true);

      //configTargetVel.setCobId(cobIdPdoTargetVel);
      //configTargetVel.setTransmissionType(hlcanopen::ASYNCHRONOUS);
      //configTargetVel.setNumberOfEntries();

      //configTargetVel.addMapping(TARGET_VELOCITY, TARGET_VELOCITY, 0x20);

      //hlcanopen::PdoConfiguration configActualVel(hlcanopen::TPDO, 1);

      //hlcanopen::COBIdPdoEntry cobIdPdoActualVel;
      //cobIdPdoActualVel.setCobId(hlcanopen::COBId(motorId, VELOCITY_ACTUAL_COD_ID));
      //cobIdPdoActualVel.enable29bitId(false);
      //cobIdPdoActualVel.enableRtr(false);
      //cobIdPdoActualVel.enablePdo(true);

      //configActualVel.setCobId(cobIdPdoActualVel);
      //configActualVel.setTransmissionType(hlcanopen::ASYNCHRONOUS);
      //configActualVel.setNumberOfEntries();

      //configActualVel.addMapping(VELOCITY_ACTUAL_VALUE, VELOCITY_ACTUAL_VALUE, 0x20);

      //manager.writePdoConfiguration(motorId, configTargetVel);
      //manager.writePdoConfiguration(motorId, configActualVel);
    }

    Td::ros_info("starting CANopen manager thread");
    canOpenManagerThread = std::thread([&](){
      manager.run();
    });
  }

  /**
   * evaluate the velocity for the left and right side wheels.
   * the velocity of a side is related to the velocity of the other side
   * by angular_v. When angular_v is 0, the velocities are the same (no turn)
   * When angular_v is 1, then we turn left and the right wheels must have
   * the opposite velocities of the left wheels.
   */
  void PowertrainManager::evaluate_velocities_m_s(double linear_v, double angular_v, float& right_v,
                           float& left_v) {
    if(angular_v > 0) {
      // turn left
      right_v = linear_v;
      left_v = linear_v * (1 - 2*angular_v);
    } else {
      // turn right
      left_v = linear_v;
      right_v = linear_v * (2*angular_v + 1);
    }
    Td::ros_info(Td::toString("evaluated velocity: [left wheels: ", left_v, "m/s] right wheels: [", right_v, " m/s]"));
  }

  void PowertrainManager::reset_motors() {
    Td::ros_info("Resetting motors");

    std::for_each(motors.begin(), motors.end(), [](Motor& m) {
      Td::ros_info(Td::toString("Set command mode for id: ", m.getId()));
      bool ok = m.setCommandMode();
      if(ok) {
        Td::ros_info(Td::toString("Command mode set for motor ", m.getId()));
      } else {
        Td::ros_warn(Td::toString("Command mode NOT set for motor ", m.getId()));
      }
    });
  }

  void PowertrainManager::printMotorsStatusWord() {
    for(Motor& m: motors) {
      int motorId = m.getId();
      StatusWord statusWord = m.getStatusWord().get();
      std::cout << "StatusWord of motor " << motorId << ":  " << statusWord << "\n" << statusWord.toStringFull() << " \n";
    }
  }

  void PowertrainManager::setControlWord(ControlWordCommand command) {
    for(Motor& m: motors) {
      int motorId = m.getId();
      Td::ros_info(Td::toString("Setting control word of motor: ", motorId));
      m.setControlWord(command);
    }
  }


  void PowertrainManager::printMotorsOperationMode() {
    for(Motor& m: motors) {
      int motorId = m.getId();
      ModeOfOperation mode = m.getOperationMode().get();
      Td::BitPrinter<4> bitPrinter;
      std::cout << "Operation mode of motor " << motorId << ": \n" << mode << " \n";
    }
  }

  void PowertrainManager::setMotorsOperationMode(ModeOfOperation mode) {
    std::for_each(motors.begin(), motors.end(), [mode](Motor& m) {
      m.setOperationMode(mode).wait();
    });
  }

  bool PowertrainManager::set_motors_enabled(bool enabled) {
    Td::ros_info(Td::toString("Set motor enabled: ", enabled));
    bool ok = true;

    for(Motor& m: motors) {
      folly::Future<folly::Unit> enableActionResult;
      if(enabled)  {
       enableActionResult = m.enable();
      } else {
       enableActionResult = m.disable();
      }

      enableActionResult.wait();

      if(enableActionResult.hasValue()) {
        Td::ros_info(Td::toString("Motor ", m.getId(), enabled ? " enabled" : " disabled"));
      } else {
        Td::ros_error(Td::toString("Motor ", m.getId(), " NOT enabled"));
        ok = false;
      }
    }

    return ok;
  }

  // Get velocity in meter per second.
  folly::Future<float> PowertrainManager::get_velocity(unsigned int motorIndex) {
    auto ans = motors[motorIndex].getVelocity();
    return ans.then([](float velRotPerS) {
        return (float)(velRotPerS * METER_PER_SECOND_REVOLUTION);
    });
  }

  bool PowertrainManager::set_velocity(double linear_v, double angular_v) {
    float right_v, left_v;

    std::cout << std::endl;
    std::cout << std::endl;
    evaluate_velocities_m_s(linear_v, angular_v, right_v, left_v);

    float right_v_rev_per_s = right_v / METER_PER_SECOND_REVOLUTION;
    float left_v_rev_per_s = left_v / METER_PER_SECOND_REVOLUTION;

    const float MIN_V = -1.0f;
    const float MAX_V = 1.0f;

    const float MAX_V_ROT = MAX_V/METER_PER_SECOND_REVOLUTION;
    const float MIN_V_ROT = MIN_V/METER_PER_SECOND_REVOLUTION;

    if(right_v > MAX_V || right_v < MIN_V) {
      Td::ros_warn(Td::toString("velocity ", right_v_rev_per_s, "m/s is not supported by right wheels. clamping"));
      right_v_rev_per_s = clamp(right_v_rev_per_s, MIN_V_ROT, MAX_V_ROT );
    }
    if(left_v > MAX_V || left_v < MIN_V) {
      Td::ros_warn(Td::toString("velocity ", left_v, "m/s is not supported by left wheels. clamping"));
      left_v_rev_per_s = clamp(left_v_rev_per_s, MIN_V_ROT, MAX_V_ROT );
    }


    for(Motor& motor : motors) {
      boost::timer t;
      if(motor.getId() == 11 || motor.getId() == 12) {
        motor.setVelocity(right_v_rev_per_s).wait();
      } else {
        motor.setVelocity(left_v_rev_per_s).wait();
      }
      std::cout<< " velocity of motor " << motor.getId() << " set in " <<  t.elapsed()  << "s" << std::endl ;
    }

    std::cout << std::endl;
    return true;
  }

  std::vector<Motor>& PowertrainManager::getMotors() {
    return motors;
  }
