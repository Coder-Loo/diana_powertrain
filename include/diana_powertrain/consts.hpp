#ifndef DIANA_POWERTRAIN_CONSTS_HPP
#define DIANA_POWERTRAIN_CONSTS_HPP

#include <hlcanopen/types.hpp>

hlcanopen::SDOIndex OS_COMMAND_MODE(0x1024, 0);
hlcanopen::SDOIndex OS_COMMAND_PROMPT_WRITE = hlcanopen::SDOIndex(0x1023, 1);
hlcanopen::SDOIndex OS_COMMAND_PROMPT_STATUS = hlcanopen::SDOIndex(0x1023, 2);
hlcanopen::SDOIndex OS_COMMAND_PROMPT_READ = hlcanopen::SDOIndex(0x1023, 3);

#endif
