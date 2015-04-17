#include "diana_powertrain/pci7841_card.hpp"

#include <hlcanopen/logging/easylogging++.h>
#include <string>
#include <iostream>
#include <iomanip>

Pci7841Card::Pci7841Card(int cardNum, int portNum)
{
  handle = CanOpenDriver(cardNum, portNum);
  if(handle < 0) {
    LOG(ERROR) << "Unable to open driver" << cardNum << ":" << portNum;
  } else {
    PORT_STRUCT setPort;
    setPort.mode = BIT11;
    setPort.accCode = 0;
    setPort.accMask = 0x7EE;
    setPort.baudrate = BAUDRATE_500KB;
    CanConfigPort(handle, &setPort);
  }
}

Pci7841Card::Pci7841Card(const Pci7841Card& r) : handle(r.handle)
{

}

Pci7841Card::~Pci7841Card()
{
  CanCloseDriver(handle);
}

bool Pci7841Card::isOk()
{
  return handle >= 0;
}

void Pci7841Card::write(const hlcanopen::CanMsg& msg)
{
  CAN_PACKET canPacket;
  memset(&canPacket, 0, sizeof(canPacket));
  canPacket.CAN_ID = msg.cobId.getCobIdValue();
  canPacket.rtr = 0;
  canPacket.len = 8;
  for (int i = 0; i < 8; ++i)
  {
    canPacket.data[i] = msg[i];
  }

  sendCanPacket(canPacket);
}

hlcanopen::CanMsg Pci7841Card::read()
{
  CAN_PACKET canPacket;
  memset(&canPacket, 0, sizeof(canPacket));


  if (CanRcvMsg(handle, &canPacket) == 0) {
    if(getCanId(canPacket.CAN_ID) == 0) {
    } else {
      LOG(DEBUG) << "receiving data: " <<
        " -- COB-ID:  " << canPacket.CAN_ID <<
        " can-id:  " << getCanId(canPacket.CAN_ID) << " -- " <<
        packetDataToStr(canPacket);
    }
  } else {
    LOG(DEBUG) << "no data received ";
    memset(&canPacket, 0, sizeof(canPacket));
  }

   hlcanopen::CanMsg canMsg;
   canMsg.cobId = hlcanopen::COBId(getCanId(canPacket.CAN_ID), getCOBType(canPacket.CAN_ID));
   for(BYTE i =0; i < 8; i++) {
      canMsg[i] = canPacket.data[i];
   }

   return canMsg;
}


void Pci7841Card::sendCanPacket(CAN_PACKET& packet)
{
    LOG(DEBUG) << "Sending to canId " << std::hex <<
      packet.CAN_ID <<  " data: " << packetDataToStr(packet);

    CanSendMsg(handle, &packet);
}

std::string Pci7841Card::packetDataToStr(const CAN_PACKET& packet)
{
  std::stringstream msg;

  msg << "<";
  for(BYTE i =0; i < 7; i++) {
    msg << std::hex << std::setfill('0') << std::setw(2) << packet.data[i] << ":";
  }
  msg << std::hex << std::setw(2) << (unsigned int) packet.data[7] << ">";

  return msg.str();
}

unsigned int Pci7841Card::getCanId(long unsigned int cobId)
{
    return cobId & 0x7F;
}

unsigned int Pci7841Card::getCOBType(long unsigned int cobId)
{
    return (cobId >> 7) & 0b1111;
}
