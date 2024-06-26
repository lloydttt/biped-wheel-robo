#ifndef TRANSIT_SEND_I_H
#define TRANSIT_SEND_I_H

#include <Arduino.h>
#include <BluetoothSerial.h>
// #include <Wire.h>
// #include <SPI.h>
#include <cmath>
#include <sstream>

BluetoothSerial SerialBT;
uint8_t address[6]={0xE4,0x65,0xB8,0x6F,0xDB,0xCE};   //从机MAC地址
void Bluetooth_Event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param); 
bool flag = false;
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
  SerialBT.begin("test2233", true);
  SerialBT.register_callback(Bluetooth_Event); //设置事件回调函数 连接 断开 发送 接收
  
  while(!SerialBT.connect(address)){
        Serial.println("Connecting...");
  }
}


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



uint16_t toHex(int a){
    // 将整数转换为十六进制字符串
    std::stringstream ss;
    ss << std::hex << std::showbase << a;
    std::string hexString = ss.str();

    // 将十六进制字符串转换为 uint16_t 类型的变量
    uint16_t hexValue;
    std::stringstream(hexString) >> std::hex >> hexValue;
    return hexValue;

}

pkg packet_create(float x, float y, float z){
    pkg temp;
    float x_intp_t;
    float x_fracp_t = modf(x, &x_intp_t);
    int x_intp = (int)x_intp_t;
    int x_fracp = x_fracp_t*1000;
    temp.Data1_I = toHex(x_intp);
    temp.Data1_F = toHex(x_fracp);
    float y_intp_t;
    float y_fracp_t = modf(y, &y_intp_t);
    int y_intp = (int)y_intp_t;
    int y_fracp = y_fracp_t*1000;
    temp.Data2_I = toHex(y_intp);
    temp.Data2_F = toHex(y_fracp);
    float z_intp_t;
    float z_fracp_t = modf(z, &z_intp_t);
    int z_intp = (int)z_intp_t;
    int z_fracp = z_fracp_t*1000;
    temp.Data3_I = toHex(z_intp);
    temp.Data3_F = toHex(z_fracp);

    return temp;
}

void Bluetooth_Event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)  //蓝牙事件回调函数
{
    if(event == ESP_SPP_OPEN_EVT || event == ESP_SPP_SRV_OPEN_EVT) //蓝牙连接成功标志 
    {                                                              //蓝牙主机和从机模式对应的标志不同，前面的是主机模式的，后面是从机模式
        Serial.write("connection successful!\r\n");
        flag = true;
    }
    else if(event == ESP_SPP_CLOSE_EVT)     //蓝牙断开连接标志
    {
        Serial.write("disconnect successful!\r\n");
        flag = false;
    }
    else if(event == ESP_SPP_DATA_IND_EVT)  //数据接收标志
    {
        while(SerialBT.available())
        {
            Serial.write(SerialBT.read());
        }
        Serial.write("  receive complete! \r\n");
    }
    else if(event == ESP_SPP_WRITE_EVT)     //数据发送标志
    {
        Serial.write("  send complete! \r\n");
    }
}


void trans_init() {
//   Serial.begin(115200);
  setupBluetooth();

}

void trans_run() {
    pkg test = packet_create(26.333, 12.333, 15.333);
    if(flag){
        sendData(&test);
    }


  // 处理其他任务
//   delay(1000);
}







#endif