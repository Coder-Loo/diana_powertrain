#include "diana_powertrain/elmo_structs.hpp"

#include <hlcanopen/logging/easylogging++.h>

#include <team_diana_lib/strings/bit_printer.h>

StatusWord::StatusWord(StatusWord::StatusWordValue value) :
value(value)
{

}

StatusWord::StatusWord(const StatusWord& oth) : value(oth.value) {
 
}

ElmoStatus StatusWord::getStatus()
{
    std::tuple<ElmoStatus, uint16_t , uint16_t> values[] = {
      //              CODE                                 MASK           VALUE
      std::make_tuple(ElmoStatus::NOT_READY_TO_SWITCH_ON,  0b0100'1111,   0b0000'0000),
      std::make_tuple(ElmoStatus::SWITCH_ON_DISABLED,      0b0100'1111,   0b0100'0000),
      std::make_tuple(ElmoStatus::READY_TO_SWITCH_ON,      0b0110'1111,   0b0010'0001),
      std::make_tuple(ElmoStatus::SWITCH_ON,               0b0110'1111,   0b0010'0011),
      std::make_tuple(ElmoStatus::OPERATION_ENABLED,       0b0110'1111,   0b0010'0111),
      std::make_tuple(ElmoStatus::QUICK_STOP_ACTIVE,       0b0110'1111,   0b0000'0111),
      std::make_tuple(ElmoStatus::FAULT_REACTION_ACTIVE,   0b0100'1111,   0b0000'1111),
      std::make_tuple(ElmoStatus::FAULT,                   0b0100'1111,   0b0000'1000)
    };

    for(auto v : values) {
      auto masked = value & std::get<1>(v);
      if(masked == std::get<2>(v)) {
          return std::get<0>(v);
      }
    }

    LOG(WARNING) << "Unkown elmo status " << value;
    return ElmoStatus::UNKNOWN;
}


StatusWord::StatusWordValue StatusWord::getValue()
{
  return value;
}


bool StatusWord::isVoltageEnabled()
{
  return value & (1 << 4);
}

bool StatusWord::isQuickStopOn()
{
  return value & (1 << 5);
}

bool StatusWord::isWarningOn()
{
  return value & (1 << 7);
}

bool StatusWord::isRemoteOn()
{
  return value & (1 << 9);
}

bool StatusWord::isTargetReached()
{
  return value & (1 << 10);
}

bool StatusWord::isInternalLimitActive()
{
  return value & (1 << 11);
}

std::string StatusWord::toStringFull()
{
  std::ostringstream os;
  os << "{\n"
     << " status: " << getStatus() << "\n"
     << " isVoltageEnabled: " << isVoltageEnabled() << "\n"
     << " isQuickStopOn: " << isQuickStopOn() << "\n"
     << " isWarningOn: " << isWarningOn() << "\n"
     << " isRemoteOn: " << isRemoteOn() << "\n"
     << " isTargetReached: " << isTargetReached() << "\n"
     << " isInternalLimitActive: " << isInternalLimitActive() << "\n"
     << "\n}";
  return os.str();
}


std::ostream& operator<< (std::ostream& os, StatusWord val) {
  std::string bitStr = Td::BitPrinter<4>().toString(val.getValue());
  return os << "StatusWord{" << bitStr << " }";
}







