#ifndef PCI7841CARD_H
#define PCI7841CARD_H

#include <hlcanopen/can_msg.hpp>

extern "C" {
  #include "pci_7841.h"
}

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
