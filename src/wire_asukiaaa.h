#ifndef _WIRE_ASUKIAAA_H_
#define _WIRE_ASUKIAAA_H_

#include <Arduino.h>
#include <Wire.h>

namespace wire_asukiaaa {
  int readBytes(TwoWire *wire, uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t dataLen);

  int writeBytes(TwoWire *wire, uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t dataLen);

  class PeripheralHandler {
  public:
    PeripheralHandler(TwoWire* wire, int buffLen, bool (*prohibitWriting)(int index) = NULL);
    ~PeripheralHandler();
    void onReceive(int);
    void onRequest();
    uint8_t* buffs;
    const int buffLen;
    unsigned long receivedAt;
    int receivedLen;

  private:
    TwoWire* wire;
    int buffIndex;
    bool (*prohibitWriting)(int index);
  };
}

#endif
