#ifndef DIANA_POWERTRAIN_CONSTS_HPP
#define DIANA_POWERTRAIN_CONSTS_HPP

#include <hlcanopen/types.hpp>

hlcanopen::SDOIndex OS_COMMAND_MODE(0x1024, 0);
hlcanopen::SDOIndex OS_COMMAND_PROMPT_WRITE = hlcanopen::SDOIndex(0x1023, 1);
hlcanopen::SDOIndex OS_COMMAND_PROMPT_STATUS = hlcanopen::SDOIndex(0x1023, 2);
hlcanopen::SDOIndex OS_COMMAND_PROMPT_READ = hlcanopen::SDOIndex(0x1023, 3);

enum  {
  SPEED_JV_LIMIT = 40000,
  // Meter per second -> JV
  MPS_JV_FACTOR = 39999 // <-- TODO: this is wrong, check actual value
} MOTOR_CONSTS;

#endif
