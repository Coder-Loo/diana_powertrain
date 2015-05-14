#ifndef DIANA_POWERTRAIN_ELMO_STRUCTS_HPP
#define DIANA_POWERTRAIN_ELMO_STRUCTS_HPP

#include "diana_powertrain/consts.hpp"

#include <ctype.h>

#include <ostream>

class StatusWord {
  using StatusWordValue = uint16_t;

public:

  StatusWord(StatusWordValue value);
  StatusWord(const StatusWord& oth);

  ElmoStatus getStatus();
  StatusWordValue getValue();
  bool isVoltageEnabled();
  bool isQuickStopOn();
  bool isWarningOn();
  bool isRemoteOn();
  bool isTargetReached();
  bool isInternalLimitActive();
  std::string toStringFull();


private:
  StatusWordValue value;
};

std::ostream& operator<< (std::ostream& os, StatusWord val);

#endif // DIANA_POWERTRAIN_ELMO_STRUCTS_HPP
