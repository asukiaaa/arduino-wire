#pragma once

#include <Arduino.h>

#ifndef WIRE_ASUKIAAA_PERI_LEN_MAX_SEND_ONCE_DEFAULT
#if defined(ARDUINO_ARCH_STM32)
#define WIRE_ASUKIAAA_PERI_LEN_MAX_SEND_ONCE_DEFAULT 32
#else
#define WIRE_ASUKIAAA_PERI_LEN_MAX_SEND_ONCE_DEFAULT 0xff
#endif
#endif

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
int readBytesByBlocks(TemplateWire* wire, uint8_t deviceAddress,
                      uint8_t registerAddress, uint8_t* data, uint8_t dataLen,
                      uint8_t blockSize,
                      const ReadConfig& readConfig = defaultReadConfig) {
  auto config = readConfig;
  for (int i = 0; blockSize * i < dataLen; ++i) {
    uint8_t lenReceived = blockSize * i;
    uint8_t regStart = registerAddress + lenReceived;
    uint8_t lenToReceive = blockSize;
    if (lenReceived + blockSize > dataLen) {
      lenToReceive = dataLen - lenReceived;
    }
    if (i != 0) {
      config.checkPresence = false;
    }
    auto result = readBytes(wire, deviceAddress, regStart, &data[lenReceived],
                            lenToReceive, config);
    if (result != 0) {
      return result;
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
        writeBytes(wire, deviceAddress, regStart, &data[lenSent], lenToSend);
    if (result != 0) {
      return result;
    }
  }
  return 0;
}

template <class TemplateWire>
class PeripheralHandlerCommonTemplate {
 public:
  void onRequest() { onRequestReturnBytesArray(); }

  void onRequestReturnOneByte() {
    if (getIsAbleToSend()) {
      getWireP()->write(getBytesSendP()[getIndexBytes()]);
      incrementIndexBytes();
    } else {
      writeEmptyValue();
    }
  }

  void onRequestReturnBytesOneByOne() {
    if (getIsAbleToSend()) {
      auto bytesSend = getBytesSendP();
      for (size_t i = 0; i < lenMaxSendOnce; ++i) {
        if (!getIsAbleToSend()) {
          break;
        }
        getWireP()->write(bytesSend[getIndexBytes()]);
        incrementIndexBytes();
      }
    } else {
      writeEmptyValue();
    }
  }

  void onRequestReturnBytesArray() {
    if (getIsAbleToSend()) {
      auto index = getIndexBytes();
      getWireP()->write(
          &getBytesSendP()[index],
          min((int)getLenBytesSend() - index, (int)lenMaxSendOnce));
    } else {
      writeEmptyValue();
    }
  }

  void setLenMaxSendOnce(uint8_t len) { lenMaxSendOnce = len; }
  uint8_t getLenMaxSendOnce() { return lenMaxSendOnce; }

 private:
  virtual TemplateWire* getWireP() = 0;
  virtual uint8_t* getBytesSendP() = 0;
  virtual uint8_t getIndexBytes() = 0;
  virtual uint8_t getLenBytesSend() = 0;
  virtual void incrementIndexBytes() = 0;
  bool getIsAbleToSend() { return getIndexBytes() < getLenBytesSend(); }
  void writeEmptyValue() { getWireP()->write((char)0); }
  uint8_t lenMaxSendOnce = WIRE_ASUKIAAA_PERI_LEN_MAX_SEND_ONCE_DEFAULT;
};

template <class TemplateWire>
class PeripheralHandlerTemplate
    : public PeripheralHandlerCommonTemplate<TemplateWire> {
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

  int getBuffIndex() { return buffIndex; }

 private:
  TemplateWire* wire;
  uint8_t buffIndex;
  bool (*prohibitWriting)(int index);
  void (*callbackOnReceiveForAddress)(uint8_t address, uint8_t data) = NULL;
  virtual void onReceiveForAddress(uint8_t address, uint8_t data) {}

  // for virtual functions
  TemplateWire* getWireP() { return wire; }
  uint8_t* getBytesSendP() { return buffs; }
  uint8_t getIndexBytes() { return buffIndex; }
  uint8_t getLenBytesSend() { return buffLen; }
  void incrementIndexBytes() { ++buffIndex; }
};

template <class TemplateWire>
class PeripheralHandlerSeparateReceiveAndSendBytesTemplate
    : public PeripheralHandlerCommonTemplate<TemplateWire> {
 public:
  uint8_t* bytesSend;
  uint8_t* bytesReceive;
  const uint8_t lenBytes;
  unsigned long receivedAt;
  void (*callbackOnReceiveForAddress)(uint8_t address, uint8_t data) = NULL;

  PeripheralHandlerSeparateReceiveAndSendBytesTemplate(TemplateWire* wire,
                                                       int lenBytes = 0xff)
      : lenBytes(lenBytes), wire(wire) {
    bytesSend = new uint8_t[lenBytes];
    bytesReceive = new uint8_t[lenBytes];
    for (int i = 0; i < lenBytes; ++i) {
      bytesSend[i] = 0;
      bytesReceive[i] = 0;
    }
    receivedAt = 0;
  }
  ~PeripheralHandlerSeparateReceiveAndSendBytesTemplate() {
    delete[] bytesSend;
    delete[] bytesReceive;
  }

  void onReceive(int) {
    uint8_t receivedLen = 0;
    while (0 < wire->available()) {
      uint8_t v = wire->read();
      if (receivedLen == 0) {
        indexBytes = v;
      } else {
        if (indexBytes < lenBytes) {
          bytesReceive[indexBytes] = v;
        }
        if (callbackOnReceiveForAddress != NULL) {
          callbackOnReceiveForAddress(indexBytes, v);
        }
        onReceiveForAddress(indexBytes, v);
        ++indexBytes;
      }
      ++receivedLen;
    }
    if (receivedLen > 0) {
      receivedAt = millis();
    }
  }

 private:
  TemplateWire* wire;
  uint8_t indexBytes;
  uint8_t buffIndex;

  virtual void onReceiveForAddress(uint8_t address, uint8_t data) {}

  // for virtual functions
  TemplateWire* getWireP() { return wire; }
  uint8_t* getBytesSendP() { return bytesSend; }
  uint8_t getIndexBytes() { return indexBytes; }
  uint8_t getLenBytesSend() { return lenBytes; }
  void incrementIndexBytes() { ++indexBytes; }
};

}  // namespace wire_asukiaaa
