#ifndef TRANS_I_H
#define TRANS_I_H

#include <Arduino.h>
// #include <BluetoothSerial.h>

// BluetoothSerial SerialBT;
// uint8_t address[6]={0xA8,0x42,0xE3,0xB4,0x1F,0xF6};   //从机MAC地址
// bool flag = false;
// void Bluetooth_Event_i(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

// void setupBluetooth() {
//   SerialBT.begin("test2233", true);
//   SerialBT.register_callback(Bluetooth_Event_i); //设置事件回调函数 连接 断开 发送 接收
  
//   while(!SerialBT.connect(address)){
//         Serial.println("Connecting...");
//   }
// }


// void trans(float v){
//     if(flag){

//        SerialBT.print(v); 

//     }
// }


// void Bluetooth_Event_i(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)  //蓝牙事件回调函数
// {
//     if(event == ESP_SPP_OPEN_EVT || event == ESP_SPP_SRV_OPEN_EVT) //蓝牙连接成功标志 
//     {                                                              //蓝牙主机和从机模式对应的标志不同，前面的是主机模式的，后面是从机模式
//         Serial.write("connection successful!\r\n");
//         flag = true;
//     }
//     else if(event == ESP_SPP_CLOSE_EVT)     //蓝牙断开连接标志
//     {
//         Serial.write("disconnect successful!\r\n");
//         flag = false;
//         while(!SerialBT.connect(address)){
//         Serial.println("Connecting...");
//         }
//     }
    
//     else if(event == ESP_SPP_DATA_IND_EVT)  //数据接收标志
//     {
//         while(SerialBT.available())
//         {
//             Serial.write(SerialBT.read());
//         }
//         Serial.write("  receive complete! \r\n");
//     }
//     else if(event == ESP_SPP_WRITE_EVT)     //数据发送标志
//     {
//         Serial.write("  send complete! \r\n");
//     }
// }
void trans(float dataToSend){
//   dataToSend = 1.17;
  byte byteArray[sizeof(float)];
  memcpy(byteArray, &dataToSend, sizeof(float));
  byte start_k = 0xAA;
  Serial2.write(start_k);
//   Serial.println(start_k);
//   Serial.println(dataToSend);
//   Serial.write(byteArray, sizeof(float));

  Serial2.write(byteArray, sizeof(float));
//   if(Serial2.available()){
//     char a = Serial2.read();
//     if(a == 'a'){
//         Serial2.flush();
//     }
//   }
//   dataToSend += 0.01;
//   if(dataToSend > 100){
//     dataToSend = 0.53;
//   }

    // Serial2.print('a');
}

























#endif