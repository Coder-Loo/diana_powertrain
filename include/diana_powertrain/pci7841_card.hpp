#ifndef PCI7841CARD_H
#define PCI7841CARD_H

#include <hlcanopen/can_msg.hpp>

extern "C" {
  #include "pci_7841.h"
}

enum Pci7841Mode {
  BIT11 = 0,
  BIT29 = 1
};

enum Pci7841Baudrate {
  BAUDRATE_125KB = 0,
  BAUDRATE_250KB = 1,
  BAUDRATE_500KB = 2,
  BAUDRATE_1MB = 3
};

class Pci7841Card {

public:

  Pci7841Card(int cardNum, int portNum);
  Pci7841Card(const Pci7841Card& r);

  ~Pci7841Card();

  bool isOk();

  hlcanopen::CanMsg read();
  void write(const hlcanopen::CanMsg& msg);

private:
  void sendCanPacket(CAN_PACKET& packet);
  std::string packetDataToStr(const CAN_PACKET& packet);
  unsigned int getCanId(long unsigned int cobId);
  unsigned int getCOBType(long unsigned int cobId);

private:
  int handle;

};

#endif // PCI7841CARD_H
