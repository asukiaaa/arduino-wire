# arduino-wire (wire_asukiaaa)

A library to supply functions or a class about i2c for Arduino.

## Usage

### Read and wirte
See [read and write](./examples/readWrite/readWrite.ino) example.

Include
```c
#include <wire_asukiaaa.h>
```

Valus for read and write example
```c
#define TARGET_DEVICE_ADDRESS 0x08
#define TARGET_WRITE_REGISTER_ADDRESS 0x00
#define TARGET_READ_REGISTER_ADDRESS 0x03
```

Write bytes.
```c
uint8_t dataToWrite[] = {0, 1, 2};
uint8_t writeLen = 3;
int writeResult = wire_asukiaaa::writeBytes(&Wire, TARGET_DEVICE_ADDRESS, TARGET_WRITE_REGISTER_ADDRESS, dataToWrite, writeLen);
if (writeResult != 0) {
   Serial.println("Failed to write because of error" + String(writeResult));
} else {
  Serial.println("Succeeded writing");
}
```

Read bytes
```c
uint8_t readLen = 3;
uint8_t dataThatRead[readLen];
int readResult = wire_asukiaaa::readBytes(&Wire, TARGET_DEVICE_ADDRESS, TARGET_READ_REGISTER_ADDRESS, dataThatRead, readLen);
if (readResult != 0) {
  Serial.println("Failed to read because of error" + String(readResult));
} else {
  Serial.println("Succeeded reading");
  Serial.print("Received:");
  for (int i = 0; i < readLen; ++i) {
    Serial.print(" ");
    Serial.print(dataThatRead[i]);
  }
  Serial.println("");
}
```

### PeripheralHandler

See [pheripheral](./examples/wirePeripheral/wirePheripheral.ino) example.
The peripheral example can comunicate with [central](./examples/central/central.ino) example.

```c
#define BUFF_LEN 10

wire_asukiaaa::PeripheralHandler wirePeri(&Wire, BUFF_LEN);
unsigned long handledReceivedAt = 0;

void setup() {
  Wire.onReceive([](int v) { wirePeri.onReceive(v); });
  Wire.onRequest([]() { wirePeri.onRequest(); });
  Wire.begin(DEVICE_ADDRESS);
}

void loop() {
  if (wirePeri.receivedAt != handledReceivedAt) {
    handledReceivedAt = wirePeri.receivedAt;
    // Do something for wirePeri.buffLen;
  }
}
```

It can prohibit writing for register by index.
```c
#define BUFF_LEN 10

bool prohibitWriting(int index) {
  // Example: Prohibit writing for last register
  return index == BUFF_LEN - 1;
}
wire_asukiaaa::PeripheralHandler wirePeri(&Wire, BUFF_LEN, prohibitWriting);
```

## License

MIT
