#pragma once

#include <Arduino.h>

namespace wire_asukiaaa {

struct ReadConfig {
  bool checkPresence = true;
  bool stopBitAfterWritingAddress = false;
};

static const ReadConfig defaultReadConfig;

template <class TemplateWire>
int checkPresence(TemplateWire* wire, uint8_t deviceAddress) {
  wire->beginTransmission(deviceAddress);
  return wire->endTransmission();
}

template <class TemplateWire>
int readBytes(TemplateWire* wire, uint8_t deviceAddress,
              uint8_t registerAddress, uint8_t* data, uint8_t dataLen,
              const ReadConfig& readConfig = defaultReadConfig) {
  uint8_t result;
  if (readConfig.checkPresence) {
    result = checkPresence(wire, deviceAddress);
    if (result != 0) {
      return result;
    }
    delay(15);
  }
  wire->beginTransmission(deviceAddress);
  wire->write(registerAddress);
  result = wire->endTransmission(readConfig.stopBitAfterWritingAddress);
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
               uint8_t registerAddress, const uint8_t* data, uint8_t dataLen) {
  wire->beginTransmission(deviceAddress);
  wire->write(registerAddress);
  wire->write(data, dataLen);
  return wire->endTransmission();
}

template <class TemplateWire>
int writeBytesByBlocks(TemplateWire* wire, uint8_t deviceAddress,
                       uint8_t registerAddress, const uint8_t* data,
                       uint8_t dataLen, uint8_t blockSize) {
  for (int i = 0; blockSize * i < dataLen; ++i) {
    uint8_t lenSent = blockSize * i;
    uint8_t regStart = registerAddress + lenSent;
    uint8_t lenToSend = blockSize;
    if (lenSent + blockSize > dataLen) {
      lenToSend = dataLen - lenSent;
    }
    auto result =
        writeBytes(wire, deviceAddress, regStart, &data[regStart], lenToSend);
    if (result != 0) {
      return result;
    }
  }
  return 0;
}

template <class TemplateWire>
class PeripheralHandlerTemplate {
 public:
  uint8_t* buffs;
  const int buffLen;
  unsigned long receivedAt;
  int receivedLen;

  PeripheralHandlerTemplate(TemplateWire* wire, int buffLen = 0xff,
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

  ~PeripheralHandlerTemplate() { delete[] buffs; }

  void setOnReceiveForAddress(void (*onReceiveForAddress)(uint8_t address,
                                                          uint8_t data)) {
    this->callbackOnReceiveForAddress = onReceiveForAddress;
  }

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
        if (callbackOnReceiveForAddress != NULL) {
          callbackOnReceiveForAddress(buffIndex, v);
        }
        onReceiveForAddress(buffIndex, v);
        ++buffIndex;
      }
      ++receivedLen;
    }
    if (receivedLen > 0) {
      receivedAt = millis();
    }
  }

  void onRequest() {
#ifdef ARDUINO_ARCH_STM32
    onRequestReturnOneByte();
#else
    onRequestReturnBytesOneByOne();
#endif
  }

  void onRequestReturnOneByte() {
    if (buffIndex < buffLen) {
      wire->write(buffs[buffIndex++]);
    } else {
      writeEmptyValue();
    }
  }

  void onRequestReturnBytesOneByOne() {
    if (buffIndex < buffLen) {
      while (buffIndex < buffLen) {
        wire->write(buffs[buffIndex++]);
      }
    } else {
      writeEmptyValue();
    }
  }

  void onRequestReturnBytesArray() {
    if (buffIndex < buffLen) {
      wire->write(&buffs[buffIndex], buffLen - buffIndex);
    } else {
      writeEmptyValue();
    }
  }

  int getBuffIndex() { return buffIndex; }

 private:
  TemplateWire* wire;
  int buffIndex;
  bool (*prohibitWriting)(int index);
  void (*callbackOnReceiveForAddress)(uint8_t address, uint8_t data) = NULL;

  virtual void onReceiveForAddress(uint8_t address, uint8_t data) {}
  void writeEmptyValue() { wire->write((char)0); }
};

}  // namespace wire_asukiaaa
