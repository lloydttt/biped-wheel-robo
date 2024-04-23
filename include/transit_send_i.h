#ifndef TRANSIT_SEND_I_H
#define TRANSIT_SEND_I_H

#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;


const byte CRC_LEN = 2; // CRC校验长度

byte dataBuffer[128]; // 数据缓存区，假设最大数据长度为128字节
int bufferIndex = 0; // 缓存区索引

// 报文结构体
struct pkg{
    const byte START_BYTE = 0xAA; // 帧头
    unsigned int data_len = sizeof(uint32_t) * 3;
    uint32_t hexData1;
    uint32_t hexData2;
    uint32_t hexData3;
    uint16_t crc;
};
// 帧头--1字节
// 数据长度




// 初始化Classic Bluetooth
void setupBluetooth() {
  SerialBT.begin("CustomProtocolExample");
}

// 发送数据，加入起始标志和CRC校验
void sendData(pkg *package) {
  // 构造带有起始标志的数据包
  byte packet[package->data_len + 4]; // 起始标志 + 数据长度 + 数据 + CRC
  packet[0] = package->START_BYTE;
  packet[1] = len;
  memcpy(packet + 2, data, len);  



  // 计算CRC并加入数据包
  uint16_t crc = calculateCRC(packet, len + 2);
  packet[len + 2] = crc & 0xFF;
  packet[len + 3] = (crc >> 8) & 0xFF;
  // 定义要传输的浮点数
    float floatData1 = 123.456;
    float floatData2 = -78.9;
    float floatData3 = 0.123;

    // 将浮点数转换为十六进制数
    uint32_t hexData1 = *(uint32_t *)&floatData1;
    uint32_t hexData2 = *(uint32_t *)&floatData2;
    uint32_t hexData3 = *(uint32_t *)&floatData3;

    // 构造数据包
    byte packet[sizeof(uint32_t) * 3]; // 三个浮点数，每个浮点数转换为四字节的十六进制数
    memcpy(packet, &hexData1, sizeof(uint32_t));
    memcpy(packet + sizeof(uint32_t), &hexData2, sizeof(uint32_t));
    memcpy(packet + sizeof(uint32_t) * 2, &hexData3, sizeof(uint32_t));

    // 发送数据包
    SerialBT.write(packet, sizeof(packet));

//   SerialBT.write(packet, len + 4);
}

// 接收数据，并校验CRC
void onDataReceived() {
  while (SerialBT.available()) {
    byte receivedByte = SerialBT.read();
    if (receivedByte == START_BYTE && bufferIndex == 0) {
      dataBuffer[bufferIndex++] = receivedByte;
    } else if (bufferIndex > 0) {
      dataBuffer[bufferIndex++] = receivedByte;
      if (bufferIndex >= CRC_LEN + 3) { // 起始标志 + 数据长度 + CRC
        byte dataLen = dataBuffer[1];
        if (bufferIndex == dataLen + 4) {
          uint16_t crcReceived = (dataBuffer[bufferIndex - 2] << 8) | dataBuffer[bufferIndex - 1];
          if (crcReceived == calculateCRC(dataBuffer, bufferIndex - 2)) {
            // CRC校验通过，处理数据
            // ...
          }
          bufferIndex = 0; // 清空缓存区
        }
      }
    }
  }
}

// 计算CRC校验值
uint16_t calculateCRC(byte *data, int len) {
  // 实现CRC校验算法，例如CRC-16
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void trans_init() {
  Serial.begin(115200);
  setupBluetooth();
}

void trans_run() {
  onDataReceived();

  // 模拟发送数据，实际应用中根据需求修改
  byte testData[] = {0x03, 0x01, 0x02}; // 示例数据
  sendData(testData, sizeof(testData));

  // 处理其他任务
  delay(1000);
}







#endif