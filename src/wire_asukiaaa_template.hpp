#ifndef _WIRE_ASUKIAAA_TEMPLATE_HPP_
#define _WIRE_ASUKIAAA_TEMPLATE_HPP_

#include <Arduino.h>

namespace wire_asukiaaa {

template <class TemplateWire>
int readBytes(TemplateWire* wire, uint8_t deviceAddress,
              uint8_t registerAddress, uint8_t* data, uint8_t dataLen) {
  wire->beginTransmission(deviceAddress);
  wire->write(registerAddress);
  uint8_t result = wire->endTransmission(false);
  if (result != 0) {
    return result;
  }

  wire->requestFrom(deviceAddress, dataLen);
  uint8_t index = 0;
  while (wire->available()) {
    uint8_t d = wire->read();
    if (index < dataLen) {
      data[index++] = d;
    }
  }
  return 0;
}

template <class TemplateWire>
int writeBytes(TemplateWire* wire, uint8_t deviceAddress,
               uint8_t registerAddress, uint8_t* data, uint8_t dataLen) {
  wire->beginTransmission(deviceAddress);
  wire->write(registerAddress);
  wire->write(data, dataLen);
  return wire->endTransmission();
}

template <class TemplateWire>
class PeripheralHandlerBase {
 public:
  uint8_t* buffs;
  const int buffLen;
  unsigned long receivedAt;
  int receivedLen;

  PeripheralHandlerBase(TemplateWire* wire, int buffLen,
                        bool (*prohibitWriting)(int index) = NULL)
      : buffLen{buffLen} {
    this->wire = wire;
    this->prohibitWriting = prohibitWriting;
    buffs = new uint8_t[buffLen];
    for (int i = 0; i < buffLen; ++i) {
      buffs[0] = 0;
    }
    receivedLen = 0;
    receivedAt = 0;
  }

  ~PeripheralHandlerBase() { delete[] buffs; }

  void onReceive(int) {
    receivedLen = 0;
    while (0 < wire->available()) {
      uint8_t v = wire->read();
      if (receivedLen == 0) {
        buffIndex = v;
      } else {
        if (buffIndex < buffLen &&
            (prohibitWriting == NULL || !prohibitWriting(buffIndex))) {
          buffs[buffIndex] = v;
        }
        ++buffIndex;
      }
      ++receivedLen;
    }
    receivedAt = millis();
  }

  void onRequest() {
    if (buffIndex < buffLen) {
      wire->write(&buffs[buffIndex], buffLen - buffIndex);
    } else {
      wire->write(0);
    }
  }

 private:
  TemplateWire* wire;
  int buffIndex;
  bool (*prohibitWriting)(int index);
};

}  // namespace wire_asukiaaa

#endif
