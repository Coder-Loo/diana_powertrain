#include "diana_powertrain/pci7841_card.hpp"

#include <hlcanopen/logging/easylogging++.h>
#include <string>
#include <iostream>
#include <iomanip>

#include <boost/assert.hpp>

Pci7841Card::Pci7841Card(int cardNum, int portNum) : cardNum(cardNum), portNum(portNum), handle(-1)
{
    std::cout << "new pci7841 card " << std::endl;
    redLed = 2*portNum;
    greenLed = 2*portNum + 1;
}

Pci7841Card::~Pci7841Card()
{
    std::cout << "destroy pci7841 card " << std::endl;
    close();
}

bool Pci7841Card::open()
{
    handle = CanOpenDriver(cardNum, portNum);
    if(handle < 0) {
        std::string msg = "Unable to open driver";
        CLOG(ERROR, "interface") << msg << cardNum << ":" << portNum;
        return false;
    } else {
        CLOG(INFO, "interface") << "p7841 card handle: " << handle;
        PORT_STRUCT setPort;
        setPort.mode = BIT11;
        setPort.accCode = 0;
        setPort.accMask = 0x7FF;
        setPort.baudrate = BAUDRATE_500KB;
        CanConfigPort(handle, &setPort);
        CanEnableReceive(handle);
        CanClearRxBuffer(handle);
        CanClearTxBuffer(handle);
        CanSetLedStatus(handle, greenLed, 1);
        CanSetLedStatus(handle, redLed, 0);
    }
    return true;
}

bool Pci7841Card::close()
{
  bool ok = true;
  if(handle >= 0) {
    CanSetLedStatus(handle, greenLed, 0);
    CanSetLedStatus(handle, redLed, 0);
    ok = CanCloseDriver(handle) == 0;
    handle = -1;
  }
  return ok;
}


bool Pci7841Card::isOk()
{
    return handle >= 0;
}

void Pci7841Card::write(const hlcanopen::CanMsg& msg)
{
    BOOST_ASSERT_MSG(isOk(), "p7841 is not initialized. Did you opened the card?");
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
    BOOST_ASSERT_MSG(isOk(), "p7841 is not initialized. Did you opened the card?");
    CAN_PACKET canPacket;
    memset(&canPacket, 0, sizeof(canPacket));

    if (CanGetRcvCnt(handle) > 0 && CanRcvMsg(handle, &canPacket) == 0) {
        auto canId = getCanId(canPacket.CAN_ID);
        CLOG(DEBUG, "interface") << "RECEIVE: " <<
                                 " -- COB-ID:  " << canPacket.CAN_ID <<
                                 " can-id: " << getCanId(canPacket.CAN_ID) << " -- data: " <<
                                 packetDataToStr(canPacket);
    } else {
        memset(&canPacket, 0, sizeof(canPacket));
    }
    CanClearRxBuffer(handle);

    hlcanopen::CanMsg canMsg;
    canMsg.cobId = hlcanopen::COBId(getCanId(canPacket.CAN_ID), getCOBType(canPacket.CAN_ID));
    for(BYTE i =0; i < 8; i++) {
        canMsg[i] = canPacket.data[i];
    }

    return canMsg;
}


void Pci7841Card::sendCanPacket(CAN_PACKET& packet)
{
    CLOG(DEBUG, "interface") << "Sending to canId " << std::hex <<
                             packet.CAN_ID <<  " data: " << packetDataToStr(packet);

    CanSendMsg(handle, &packet);
}

std::string Pci7841Card::packetDataToStr(const CAN_PACKET& packet)
{
    std::stringstream msg;
    int size = std::min(packet.len, (unsigned char) 8);

    msg << "at : "<< &packet.data << " <";
    for(int i =0; i < size-1; i++) {
        msg << std::hex << std::setw(2) << std::setfill('0')  << (unsigned int) packet.data[i] << ":";
    }
    msg << std::hex << std::setw(2) << std::setfill('0') << (unsigned int) packet.data[size-1] << ">";

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
