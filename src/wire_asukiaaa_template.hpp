#pragma once

#include <Arduino.h>

#ifndef WIRE_ASUKIAAA_PERI_LEN_MAX_SEND_ONCE_DEFAULT
#if defined(ARDUINO_ARCH_STM32) || defined(TEENSYDUINO)
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
  unsigned long receivedAt = 0;
  int receivedLen = 0;

  void onReceive(int) {
    receivedLen = 0;
    auto bytesReceive = getBytesReceiveP();
    while (0 < wire->available()) {
      uint8_t v = wire->read();
      if (receivedLen == 0) {
        indexBytes = v;
      } else {
        if (indexBytes < getLenBytesSend() &&
            (prohibitWriting == NULL || !prohibitWriting(indexBytes))) {
          bytesReceive[indexBytes] = v;
        }
        if (PeripheralHandlerCommonTemplate<
                TemplateWire>::callbackOnReceiveForAddress != NULL) {
          PeripheralHandlerCommonTemplate<
              TemplateWire>::callbackOnReceiveForAddress(indexBytes, v);
        }
        PeripheralHandlerCommonTemplate<TemplateWire>::onReceiveForAddress(
            indexBytes, v);
        ++indexBytes;
      }
      ++receivedLen;
    }
    if (receivedLen > 0) {
      receivedAt = millis();
    }
  }

  void onRequest() { onRequestReturnBytesArray(); }

  void onRequestReturnOneByte() {
    onSendFromAddress(indexBytes);
    if (getIsAbleToSend()) {
      wire->write(getBytesSendP()[indexBytes++]);
    } else {
      writeEmptyValue();
    }
  }

  void onRequestReturnBytesOneByOne() {
    onSendFromAddress(indexBytes);
    if (getIsAbleToSend()) {
      auto bytesSend = getBytesSendP();
      for (size_t i = 0; i < lenMaxSendOnce; ++i) {
        if (!getIsAbleToSend()) {
          break;
        }
        wire->write(bytesSend[indexBytes++]);
      }
    } else {
      writeEmptyValue();
    }
  }

  void onRequestReturnBytesArray() {
    onSendFromAddress(indexBytes);
    if (getIsAbleToSend()) {
      auto lenSend =
          min((int)getLenBytesSend() - indexBytes, (int)lenMaxSendOnce);
      wire->write(&getBytesSendP()[indexBytes], lenSend);
      indexBytes += lenSend;
    } else {
      writeEmptyValue();
    }
  }

  void setLenMaxSendOnce(uint8_t len) { lenMaxSendOnce = len; }
  uint8_t getLenMaxSendOnce() { return lenMaxSendOnce; }
  void setOnReceiveForAddress(void (*onReceiveForAddress)(uint8_t address,
                                                          uint8_t data)) {
    this->callbackOnReceiveForAddress = onReceiveForAddress;
  }

 protected:
  TemplateWire* wire;
  bool (*prohibitWriting)(int index) = NULL;

 private:
  // required virtual functions
  virtual uint8_t* getBytesReceiveP() = 0;
  virtual uint8_t* getBytesSendP() = 0;
  virtual uint8_t getLenBytesSend() = 0;

  uint8_t indexBytes;

  bool getIsAbleToSend() { return indexBytes < getLenBytesSend(); }
  void writeEmptyValue() { wire->write((char)0); }
  uint8_t lenMaxSendOnce = WIRE_ASUKIAAA_PERI_LEN_MAX_SEND_ONCE_DEFAULT;

  void (*callbackOnReceiveForAddress)(uint8_t address, uint8_t data) = NULL;
  virtual void onReceiveForAddress(uint8_t address, uint8_t data) {}
  virtual void onSendFromAddress(uint8_t address){};
};

template <class TemplateWire>
class PeripheralHandlerTemplate
    : public PeripheralHandlerCommonTemplate<TemplateWire> {
 public:
  uint8_t* buffs;
  const int buffLen;

  PeripheralHandlerTemplate(TemplateWire* wire, int buffLen = 0xff,
                            bool (*prohibitWriting)(int index) = NULL)
      : buffLen{buffLen} {
    PeripheralHandlerCommonTemplate<TemplateWire>::wire = wire;
    PeripheralHandlerCommonTemplate<TemplateWire>::prohibitWriting =
        prohibitWriting;
    buffs = new uint8_t[buffLen];
    for (int i = 0; i < buffLen; ++i) {
      buffs[0] = 0;
    }
  }

  ~PeripheralHandlerTemplate() { delete[] buffs; }

  int getBuffIndex() {
    return PeripheralHandlerCommonTemplate<TemplateWire>::indexBytes;
  }

 private:
  // for virtual functions
  uint8_t* getBytesReceiveP() { return buffs; }
  uint8_t* getBytesSendP() { return buffs; }
  uint8_t getLenBytesSend() { return buffLen; }
};

template <class TemplateWire>
class PeripheralHandlerSeparateReceiveAndSendBytesTemplate
    : public PeripheralHandlerCommonTemplate<TemplateWire> {
 public:
  uint8_t* bytesSend;
  uint8_t* bytesReceive;
  const uint8_t lenBytes;

  PeripheralHandlerSeparateReceiveAndSendBytesTemplate(TemplateWire* wire,
                                                       int lenBytes = 0xff)
      : lenBytes(lenBytes) {
    PeripheralHandlerCommonTemplate<TemplateWire>::wire = wire;
    bytesSend = new uint8_t[lenBytes];
    bytesReceive = new uint8_t[lenBytes];
    for (int i = 0; i < lenBytes; ++i) {
      bytesSend[i] = 0;
      bytesReceive[i] = 0;
    }
  }
  ~PeripheralHandlerSeparateReceiveAndSendBytesTemplate() {
    delete[] bytesSend;
    delete[] bytesReceive;
  }

 private:
  // for virtual functions
  uint8_t* getBytesReceiveP() { return bytesReceive; }
  uint8_t* getBytesSendP() { return bytesSend; }
  uint8_t getLenBytesSend() { return lenBytes; }
};

}  // namespace wire_asukiaaa
