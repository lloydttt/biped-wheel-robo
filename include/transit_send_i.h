#ifndef TRANSIT_SEND_I_H
#define TRANSIT_SEND_I_H

#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;


// const byte CRC_LEN = 2; // CRC校验长度

// byte dataBuffer[128]; // 数据缓存区，假设最大数据长度为128字节
// int bufferIndex = 0; // 缓存区索引

// // 报文结构体
struct pkg {
    const uint16_t START_BYTE = 0xAA; // 帧头
    uint16_t data_len = 0xC;   // 数据长度
    uint16_t Data1_I;    // 数据1整位
    uint16_t Data1_F;    // 数据1浮点位
    uint16_t Data2_I;
    uint16_t Data2_F;
    uint16_t Data3_I;
    uint16_t Data3_F;
};


// 初始化Classic Bluetooth
void setupBluetooth() {
  SerialBT.begin();
}

// // 发送数据，加入起始标志和CRC校验
// void sendData(pkg *package) {
//   // 构造带有起始标志的数据包
//   uint16_t packet[10]; // 起始标志 + 数据长度 + 数据 + CRC
//   // packet[0] = package->START_BYTE;
//   // packet[1] = package->data_len;
//   memcpy(packet, package, sizeof(uint16_t)*8);  


//   // 计算CRC并加入数据包
//   uint16_t crc = calculateCRC(packet, 8);
//   packet[8] = crc & 0xFF;
//   packet[9] = (crc >> 8) & 0xFF;

//     // 发送数据包
//     SerialBT.write((uint8_t *)&packet, size_t(sizeof(uint16_t)*10));
//     // Serial.print(packet[1]);
// //   SerialBT.write(packet, len + 4);
// }

void sendData(pkg *package) {
    // 构造带有起始标志的数据包
    uint16_t packet[10]; // 起始标志 + 数据长度 + 数据 + CRC
    packet[0] = package->START_BYTE;
    packet[1] = package->data_len;
    packet[2] = package->Data1_I;
    packet[3] = package->Data1_F;
    packet[4] = package->Data2_I;
    packet[5] = package->Data2_F;
    packet[6] = package->Data3_I;
    packet[7] = package->Data3_F;

    // 计算CRC并加入数据包
    uint16_t crc = calculateCRC(packet, 8);
    packet[8] = crc & 0xFF;
    packet[9] = (crc >> 8) & 0xFF;

    // 发送数据包
    SerialBT.write((uint8_t *)packet, sizeof(packet));
}
// // 接收数据，并校验CRC--------------------------------------------------------------------------------------------
// // void onDataReceived() {
// //   while (SerialBT.available()) {
// //     byte receivedByte = SerialBT.read();
// //     if (receivedByte == START_BYTE && bufferIndex == 0) {
// //       dataBuffer[bufferIndex++] = receivedByte;
// //     } else if (bufferIndex > 0) {
// //       dataBuffer[bufferIndex++] = receivedByte;
// //       if (bufferIndex >= CRC_LEN + 3) { // 起始标志 + 数据长度 + CRC
// //         byte dataLen = dataBuffer[1];
// //         if (bufferIndex == dataLen + 4) {
// //           uint16_t crcReceived = (dataBuffer[bufferIndex - 2] << 8) | dataBuffer[bufferIndex - 1];
// //           if (crcReceived == calculateCRC(dataBuffer, bufferIndex - 2)) {
// //             // CRC校验通过，处理数据
// //             // ...
// //           }
// //           bufferIndex = 0; // 清空缓存区
// //         }
// //       }
// //     }
// //   }
// // }
// // ------------------------------------------------------------------------------------------------------------------------------
// // 计算CRC校验值
uint16_t calculateCRC(uint16_t *data, int len) {
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

pkg packet_create(float x, float y, float z){
    int x_intp;
    
}

void trans_init() {
  Serial.begin(115200);
  setupBluetooth();
}

void trans_run() {
  // onDataReceived();

  // 模拟发送数据，实际应用中根据需求修改
  // byte testData[] = {0x03, 0x01, 0x02}; // 示例数据
  // sendData(testData, sizeof(testData));

  // 处理其他任务
//   delay(1000);
}







#endif