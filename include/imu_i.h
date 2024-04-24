// #include "EEPROM.h"
// #include <Arduino.h>
// #include <U8g2lib.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps_V6_12.h"
// #include <SimpleFOC.h>
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// #include <Wire.h>
// #include <SPI.h>
// #endif

// #define LPF_a 0.7 // 滤波系数a(0-1)

// #define k1pin 25 //按键IO
// #define k2pin 26
// #define k3pin 27

// #define INTERRUPT_PIN 23 // 陀螺仪外部中断io
// MPU6050 mpu;

// Commander command = Commander(Serial);

// float A1_angle = 450; // 初始目标角度
// float A2_angle = 350;
// float A3_angle = 290;
// float B1_angle = 400;
// float B2_angle = 350;
// float B3_angle = 350;

// //   舵机定义封装通用回调
// //输入值即为
// void onA1A(char *cmd)
// {
//   float map_angle = atoi(cmd);
//   if (map_angle >= 0 && map_angle <= 900)
//   {
//     A1_angle = map_angle;
//     Serial.print("A1:");
//     Serial.println(A1_angle);
//   }
// }
// void onA2A(char *cmd)
// {
//   float map_angle = atoi(cmd);
//   if (map_angle >= 0 && map_angle <= 900)
//   {
//     A2_angle = map_angle;
//         Serial.print("A2:");
//     Serial.println(A2_angle);
//   }
// }
// void onA3A(char *cmd)
// {
//   float map_angle = atoi(cmd);
//   if (map_angle >= 0 && map_angle <= 900)
//   {
//     A3_angle = map_angle;
//         Serial.print("A3:");
//     Serial.println(A3_angle);
//   }
// }
// void onB1A(char *cmd)
// {
//   float map_angle = atoi(cmd);
//   if (map_angle >= 0 && map_angle <= 900)
//   {
//     B1_angle = map_angle;
//         Serial.print("B1:");
//     Serial.println(B1_angle);
//   }
// }
// void onB2A(char *cmd)
// {
//   float map_angle = atoi(cmd);
//   if (map_angle >= 0 && map_angle <= 900)
//   {
//     B2_angle = map_angle;
//         Serial.print("B2:");
//     Serial.println(B2_angle);
//   }
// }
// void onB3A(char *cmd)
// {
//   float map_angle = atoi(cmd);
//   if (map_angle >= 0 && map_angle <= 900)
//   {
//     B3_angle = map_angle;
//         Serial.print("B3:");
//     Serial.println(B3_angle);
//   }
// }
// // void onA2A(char* cmd) { command.motor(&A2_angle, cmd); }
// // void onA3A(char* cmd) { command.motor(&A3_angle, cmd); }
// // void onB1A(char* cmd) { command.motor(&B1_angle, cmd); }
// // void onB2A(char* cmd) { command.motor(&B2_angle, cmd); }
// // void onB3A(char* cmd) { command.motor(&B3_angle, cmd); }

// float LPF_value; //滤波后的值

// int show_f = 2; //显示标志位
// int start_f = 0;
// int jdbz = 0;
// char jdbz_press_time = 0;
// char jdbz_press_time1 = 0;
// float Ts = 0; //周期时间
// unsigned long Tt;
// unsigned long now_us = 0;
// unsigned long velocity_calc_timestamp = 0;

// struct
// {

//   float Q_angle;
//   float Q_bias;
//   float R_measure;
//   float Angle;
//   float Angle1;
//   float Bias;
//   float Rate;
//   float Rate1;
//   float Rate2;

//   float p[2][2];

// } KalmanX, KalmanY, KalmanZ;

// /* IMU Data */
// bool imuReady = false; //
// float AngleZ = 0, AngleX = 0, AngleY = 0;
// float AngleX_bias = 0, AngleY_bias = 0;
// bool dmpReady = false;
// uint8_t mpuIntStatus;
// uint8_t devStatus;
// uint16_t packetSize;
// uint8_t fifoBuffer[64];

// // orientation/motion vars
// Quaternion q;
// VectorInt16 aa;
// VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
// VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity; // [x, y, z]            gravity vector
// float euler[3];
// float ypr[3];

// volatile bool mpuInterrupt = false;

// boolean key1, key2, key3;
// boolean s1, s2, s3;
// boolean s1_long_press, s2_long_press, s3_long_press;
// char s1_press_time, s2_press_time, s3_press_time, ch5_press_time;
// char menu_f = 0;

// //舵机初始设置....................................................................................................................................................................
// #define A1_PWM 15 //舵机IO
// #define A2_PWM 2
// #define A3_PWM 0
// #define B1_PWM 13
// #define B2_PWM 33
// #define B3_PWM 32

// int freq = 50;        // 频率
// int resolution0 = 20; // 分辨率，取值0~20，duty最大取值为2^resolution-1
// int channel0 = 0;     // 通道0，共16个通道，0~15
// int channel1 = 1;     //
// int channel2 = 2;     //
// int channel3 = 3;     //
// int channel4 = 4;     //
// int channel5 = 5;     //

// float B_Angle = 0; //设置机器高度
// float A_Angle = 0;

// int A1angle_max = 79999; //设置舵机角度的最大值
// int A2angle_max = 79999;
// int A3angle_max = 79999;
// int A1angle_mini = 29999;
// int A2angle_mini = 29999;
// int A3angle_mini = 29999;

// int B1angle_max = 79999;
// int B2angle_max = 79999;
// int B3angle_max = 79999;
// int B1angle_mini = 129999;
// int B2angle_mini = 129999;
// int B3angle_mini = 129999;
// //.............................................................................................................................................................................

// uint8_t n_sample = 8;        //滑动加权滤波算法采样个数
// float sample_array[8] = {0}; //采样队列

// uint8_t n_sample1 = 8;
// float sample_array1[8] = {0};

// uint8_t n_sample2 = 8;
// float sample_array2[8] = {0};

// uint8_t n_sample3 = 8;
// float sample_array3[8] = {0};

// /////////////////////////////////
// int ix, i1, i2, i3, tx, t0, t1, sta;
// float V_min, V_max, V_mid;
// long Freq;
// float Vpp;
// float Y[96]; //声明信号值储存数组
// float Buffer[192];
// const float L[] PROGMEM = {
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x03, 0x70, 0x3F, 0x87, 0xF0, 0xC1, 0x99, 0x83,
//     0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x07, 0x30, 0x3F, 0xC7, 0xF8, 0xC1, 0x99, 0xC3,
//     0x1F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x06, 0x38, 0x30, 0xC6, 0x1C, 0xC1, 0x99, 0xE3,
//     0x18, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x0E, 0x18, 0x30, 0xC6, 0x0C, 0xC1, 0x99, 0xF3,
//     0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x0F, 0xFC, 0x3F, 0xC6, 0x0C, 0xC1, 0x99, 0xBB,
//     0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x1F, 0xFC, 0x3F, 0x06, 0x0C, 0xC1, 0x99, 0x9F,
//     0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x18, 0x0E, 0x31, 0x86, 0x1C, 0xE3, 0x99, 0x8F,
//     0x38, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x38, 0x07, 0x30, 0xC7, 0xF8, 0x7F, 0x19, 0x87,
//     0x1F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x78, 0x07, 0xB0, 0xE7, 0xE0, 0x3E, 0x19, 0x87,
//     0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x00, 0x10,
//     0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x04, 0x92, 0x00, 0x08,
//     0x00, 0x00, 0x20, 0x40, 0x80, 0x3E, 0x78, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x02, 0xA4, 0x01, 0xFF,
//     0xE0, 0x7F, 0xC0, 0x40, 0x80, 0x24, 0x88, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x02, 0x00,
//     0x40, 0x00, 0x00, 0x08, 0x88, 0x24, 0x88, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x0F, 0xF7, 0xE2, 0x00,
//     0x80, 0x00, 0x00, 0x1F, 0xF8, 0x3C, 0x78, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x01, 0xCC, 0x81, 0xFF,
//     0x80, 0x00, 0x19, 0xA8, 0x90, 0x26, 0xA0, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x02, 0xB4, 0x80, 0x02,
//     0x03, 0xFF, 0xE0, 0xA8, 0x80, 0x03, 0x14, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x04, 0x94, 0x80, 0x04,
//     0x00, 0x24, 0x00, 0x27, 0xF8, 0x7C, 0xE8, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x08, 0x84, 0x80, 0x0C,
//     0x00, 0x24, 0x80, 0x4A, 0x10, 0x0C, 0x40, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x01, 0x34, 0x87, 0xFF,
//     0xF0, 0x44, 0x40, 0x49, 0x20, 0x10, 0x30, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x07, 0xE3, 0x00, 0x08,
//     0x00, 0x44, 0x20, 0x49, 0x20, 0x7E, 0xFE, 0x00,
//     0x00, 0x1E, 0x00, 0x00, 0x02, 0x43, 0x00, 0x08,
//     0x00, 0x84, 0x10, 0x88, 0xC0, 0x22, 0x88, 0x00,
//     0x00, 0x33, 0x00, 0x00, 0x01, 0xC3, 0x00, 0x08,
//     0x01, 0x04, 0x10, 0x90, 0xC0, 0x22, 0x88, 0x00,
//     0x00, 0x61, 0x80, 0x00, 0x01, 0xB4, 0x80, 0x08,
//     0x02, 0x04, 0x00, 0x91, 0x30, 0x22, 0x88, 0x00,
//     0x00, 0xC0, 0x80, 0x00, 0x02, 0x08, 0x60, 0x38,
//     0x00, 0x1C, 0x00, 0xA6, 0x1C, 0x3E, 0xF8, 0x00,
//     0x00, 0x80, 0xC0, 0x00, 0x04, 0x30, 0x00, 0x10,
//     0x00, 0x08, 0x00, 0x58, 0x00, 0x00, 0x00, 0x00,
//     0x01, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x01, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x01, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x03, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x02, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x02, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x03,
//     0x00, 0x60, 0x00, 0x20, 0x00, 0x18, 0x00, 0x00,
//     0x06, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x06,
//     0x00, 0x30, 0x00, 0x60, 0x00, 0x18, 0x00, 0x00,
//     0x06, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x06,
//     0x00, 0x30, 0x00, 0x60, 0x00, 0x18, 0x00, 0x00,
//     0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x06,
//     0x1E, 0x31, 0xBC, 0xFE, 0x38, 0xDB, 0xC1, 0xD8,
//     0x04, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x0C,
//     0x3F, 0x19, 0xFE, 0xFE, 0x38, 0xDF, 0xE3, 0xF8,
//     0x04, 0x00, 0x18, 0x00, 0x10, 0x00, 0x00, 0x0C,
//     0x73, 0x19, 0xC6, 0x63, 0x39, 0x9C, 0x67, 0x38,
//     0x0C, 0x00, 0x18, 0x00, 0x18, 0x00, 0x00, 0x0C,
//     0x60, 0x19, 0x86, 0x63, 0x6D, 0x98, 0x66, 0x18,
//     0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x0C,
//     0x60, 0x19, 0x86, 0x63, 0x6D, 0x98, 0x66, 0x18,
//     0x0C, 0x00, 0x08, 0x00, 0x10, 0x00, 0x00, 0x0C,
//     0x60, 0x19, 0x86, 0x63, 0x6D, 0x98, 0x66, 0x18,
//     0x00, 0x00, 0x08, 0x00, 0x30, 0x00, 0x00, 0x0C,
//     0x73, 0x19, 0x86, 0x61, 0xC7, 0x18, 0x67, 0x38,
//     0x00, 0x00, 0x0C, 0x00, 0x30, 0x00, 0x00, 0x0C,
//     0x3F, 0x19, 0x86, 0x79, 0xC7, 0x18, 0x63, 0xF8,
//     0x00, 0x00, 0x0C, 0x00, 0x30, 0x00, 0x00, 0x06,
//     0x1E, 0x31, 0x86, 0x39, 0xC7, 0x18, 0x61, 0xD8,
//     0x00, 0x00, 0x04, 0x00, 0x20, 0x00, 0x00, 0x06,
//     0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
//     0x00, 0x00, 0x04, 0x00, 0x20, 0x00, 0x00, 0x06,
//     0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
//     0x00, 0x00, 0x06, 0x00, 0x60, 0x00, 0x00, 0x03,
//     0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
//     0x00, 0x00, 0x06, 0x00, 0x60, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x02, 0x00, 0x40, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x03, 0x00, 0xC0, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x01, 0xC7, 0x04, 0x70, 0x38, 0xE0,
//     0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x02, 0x28, 0x8C, 0x88, 0x45, 0x10,
//     0x00, 0x00, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x28, 0x94, 0x08, 0x45, 0x10,
//     0x00, 0x00, 0x01, 0x81, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x48, 0x84, 0x30, 0x44, 0xE0,
//     0x00, 0x00, 0x00, 0xC3, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x88, 0x84, 0x08, 0x45, 0x10,
//     0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x01, 0x08, 0x84, 0x88, 0x45, 0x10,
//     0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x03, 0xE7, 0x04, 0x70, 0x38, 0xE0,
//     0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// /////////////////////////////////

// struct
// {

//   float Q_Xk;

//   float R_measure;

//   float Xk;

//   float p1;

// } A_speed, B_speed, Zturn;

// void dmpDataReady() //陀螺仪外部中断
// {
//   mpuInterrupt = true; //陀螺仪中断过标志位
// }

// void Measure()
// {
//   V_max = Buffer[0];
//   V_min = Buffer[0];
//   for (ix = 0; ix < 192; ix++)
//   {
//     if (Buffer[ix] > V_max)
//       V_max = Buffer[ix];
//     if (Buffer[ix] < V_min)
//       V_min = Buffer[ix];
//   }
//   V_mid = (V_max + V_min) / 2;
//   Vpp = V_max - V_mid;
//   for (ix = 0; ix < 97; ix++)
//   {
//     if (Buffer[ix] < V_mid && Buffer[ix + 1] >= V_mid)
//     {
//       i1 = ix;
//       break;
//     }
//   }
//   for (ix = i1 + 1; ix < 98 + i1; ix++)
//   {
//     if (Buffer[ix] < V_mid && Buffer[ix + 1] >= V_mid)
//     {
//       i2 = ix;
//       break;
//     }
//   }
//   tx = i2 - i1;
//   if (tx > 0)
//     Freq = 1 / (tx * 0.01);
//   else
//     Freq = 0;
// }

// void Kalman_init()
// {
//   KalmanY.Q_angle = 0.001f;
//   KalmanY.Q_bias = 0.003f;
//   KalmanY.R_measure = 0.035f;
//   KalmanY.Angle = 0.0f;
//   KalmanY.Bias = 0.0f;
//   KalmanY.p[0][0] = 0.0f;
//   KalmanY.p[0][1] = 0.0f;
//   KalmanY.p[1][0] = 0.0f;
//   KalmanY.p[1][1] = 0.0f;

//   KalmanX.Q_angle = 0.001f;
//   KalmanX.Q_bias = 0.003f;
//   KalmanX.R_measure = 0.035f;
//   KalmanX.Angle = 0.0f;
//   KalmanX.Bias = 0.0f;
//   KalmanX.p[0][0] = 0.0f;
//   KalmanX.p[0][1] = 0.0f;
//   KalmanX.p[1][0] = 0.0f;
//   KalmanX.p[1][1] = 0.0f;

//   KalmanZ.Q_angle = 0.001f;
//   KalmanZ.Q_bias = 0.003f;
//   KalmanZ.R_measure = 0.035f;
//   KalmanZ.Angle = 0.0f;
//   KalmanZ.Bias = 0.0f;
//   KalmanZ.p[0][0] = 0.0f;
//   KalmanZ.p[0][1] = 0.0f;
//   KalmanZ.p[1][0] = 0.0f;
//   KalmanZ.p[1][1] = 0.0f;
// }

// void YKalmangGetAngle(float newAngle, float newRate, float dt)
// {

//   KalmanY.Rate = newRate - KalmanY.Bias;
//   KalmanY.Angle += dt * KalmanY.Rate;

//   KalmanY.p[0][0] += dt * (dt * KalmanY.p[1][1] - KalmanY.p[0][1] - KalmanY.p[1][0] + KalmanY.Q_angle);
//   KalmanY.p[0][1] -= dt * KalmanY.p[1][1];
//   KalmanY.p[1][0] -= dt * KalmanY.p[1][1];
//   KalmanY.p[1][1] += KalmanY.Q_bias * dt;

//   float S = KalmanY.p[0][0] + KalmanY.R_measure; // Estimate error

//   float K[2]; // Kalman gain - This is a 2x1 vector
//   K[0] = KalmanY.p[0][0] / S;
//   K[1] = KalmanY.p[1][0] / S;

//   float y = newAngle - KalmanY.Angle; // Angle difference

//   KalmanY.Angle += K[0] * y;
//   KalmanY.Bias += K[1] * y;

//   float P00_temp = KalmanY.p[0][0];
//   float P01_temp = KalmanY.p[0][1];

//   KalmanY.p[0][0] -= K[0] * P00_temp;
//   KalmanY.p[0][1] -= K[0] * P01_temp;
//   KalmanY.p[1][0] -= K[1] * P00_temp;
//   KalmanY.p[1][1] -= K[1] * P01_temp;
// }

// void XKalmangGetAngle(float newAngle, float newRate, float dt)
// {

//   KalmanX.Rate = newRate - KalmanX.Bias;
//   KalmanX.Angle += dt * KalmanX.Rate;

//   KalmanX.p[0][0] += dt * (dt * KalmanX.p[1][1] - KalmanX.p[0][1] - KalmanX.p[1][0] + KalmanX.Q_angle);
//   KalmanX.p[0][1] -= dt * KalmanX.p[1][1];
//   KalmanX.p[1][0] -= dt * KalmanX.p[1][1];
//   KalmanX.p[1][1] += KalmanX.Q_bias * dt;

//   float S = KalmanX.p[0][0] + KalmanX.R_measure; // Estimate error

//   float K[2]; // Kalman gain - This is a 2x1 vector
//   K[0] = KalmanX.p[0][0] / S;
//   K[1] = KalmanX.p[1][0] / S;

//   float y = newAngle - KalmanX.Angle; // Angle difference

//   KalmanX.Angle += K[0] * y;
//   KalmanX.Bias += K[1] * y;

//   float P00_temp = KalmanX.p[0][0];
//   float P01_temp = KalmanX.p[0][1];

//   KalmanX.p[0][0] -= K[0] * P00_temp;
//   KalmanX.p[0][1] -= K[0] * P01_temp;
//   KalmanX.p[1][0] -= K[1] * P00_temp;
//   KalmanX.p[1][1] -= K[1] * P01_temp;
// }

// void ZKalmangGetAngle(float newAngle, float newRate, float dt)
// {

//   KalmanZ.Rate = newRate - KalmanZ.Bias;
//   KalmanZ.Angle += dt * KalmanZ.Rate;

//   KalmanZ.p[0][0] += dt * (dt * KalmanZ.p[1][1] - KalmanZ.p[0][1] - KalmanZ.p[1][0] + KalmanZ.Q_angle);
//   KalmanZ.p[0][1] -= dt * KalmanZ.p[1][1];
//   KalmanZ.p[1][0] -= dt * KalmanZ.p[1][1];
//   KalmanZ.p[1][1] += KalmanZ.Q_bias * dt;

//   float S = KalmanZ.p[0][0] + KalmanZ.R_measure; // Estimate error

//   float K[2]; // Kalman gain - This is a 2x1 vector
//   K[0] = KalmanZ.p[0][0] / S;
//   K[1] = KalmanZ.p[1][0] / S;

//   float y = newAngle - KalmanZ.Angle; // Angle difference

//   KalmanZ.Angle += K[0] * y;
//   KalmanZ.Bias += K[1] * y;

//   float P00_temp = KalmanZ.p[0][0];
//   float P01_temp = KalmanZ.p[0][1];

//   KalmanZ.p[0][0] -= K[0] * P00_temp;
//   KalmanZ.p[0][1] -= K[0] * P01_temp;
//   KalmanZ.p[1][0] -= K[1] * P00_temp;
//   KalmanZ.p[1][1] -= K[1] * P01_temp;
// }

// void EEPROM_init()
// {
//   if (!EEPROM.begin(101))
//   {
//     Serial.println("Failed to initialise EEPROM");
//     Serial.println("Restarting...");
//     delay(1000);
//     ESP.restart();
//   }
//   AngleX_bias = EEPROM.readFloat(30);
//   AngleY_bias = EEPROM.readFloat(40);
//   Serial.print("AngleX_bias:");
//   Serial.print(AngleX_bias);
//   Serial.print("  ---  ");
//   Serial.print("AngleY_bias:");
//   Serial.println(AngleY_bias);
// }

// float IMU_Sliding_weighted_filter(float xdat3) //滑动加权滤波算法
// {
//   long array_sum = 0; //采样队列和
//   for (int i = 1; i < n_sample3; i++)
//   {
//     sample_array3[i - 1] = sample_array3[i];
//     array_sum += sample_array3[i] * i;
//   }
//   sample_array3[n_sample3 - 1] = xdat3;
//   array_sum += xdat3 * n_sample3;
//   float filte_value = (array_sum / (11 * n_sample3 / 2.0)) * 9 / 7.0; //
//   return filte_value;
// }

// float LPF_filter(float x) //低通滤波
// {
//   LPF_value = LPF_a * LPF_value + (1 - LPF_a) * x;
//   return LPF_value;
// }

// boolean crc1(unsigned char buffer[])
// {
//   unsigned int crc_bit1 = 0;
//   unsigned int sum1 = 0;

//   for (int j = 2; j <= 5; j++)
//   {
//     sum1 += buffer[j];
//   }
//   crc_bit1 = sum1 & 0xff;
//   if ((unsigned char)crc_bit1 == buffer[6])
//     return true;
//   else
//     return false;
// }

// unsigned char crc2(unsigned char buffer[])
// {
//   unsigned int crc_bit1 = 0;
//   unsigned int sum1 = 0;

//   for (int j = 2; j <= 5; j++)
//   {
//     sum1 += buffer[j];
//   }
//   crc_bit1 = sum1 & 0xff;

//   return (unsigned char)crc_bit1;
// }

// //舵机角度设置//////////////////////////////////////////////////////////////////////////////////////////////////////////

// void Set_servo_Angle(float aangle, float bangle)
// {

//   float a_angle = aangle;
//   float b_angle = bangle;

//   if (a_angle < 0)
//     a_angle = 0;
//   if (a_angle > 800)
//     a_angle = 800;

//   if (b_angle < 0)
//     b_angle = 0;
//   if (b_angle > 800)
//     b_angle = 800;

//   /*ledcWrite(channel0,map(b_angle, 0, 900, B1angle_mini, B1angle_max));  //B1
//   ledcWrite(channel1,map(b_angle, 0, 900, B2angle_mini, B2angle_max));  //B2
//   ledcWrite(channel2,map(b_angle, 0, 900, B3angle_mini, B3angle_max));  //B3
//   ledcWrite(channel3,map(a_angle, 0, 900, A1angle_mini, A1angle_max));  //A1
//   ledcWrite(channel4,map(a_angle, 0, 900, A2angle_mini, A2angle_max));  //A2
//   ledcWrite(channel5,map(a_angle, 0, 900, A3angle_mini, A3angle_max));  //A3
//   */
// }

// void Servo_Initialization() // ledc初始化程序
// {
//   ledcSetup(channel0, freq, resolution0); // 设置通道0
//   ledcSetup(channel1, freq, resolution0); //
//   ledcSetup(channel2, freq, resolution0); //
//   ledcSetup(channel3, freq, resolution0); //
//   ledcSetup(channel4, freq, resolution0); //
//   ledcSetup(channel5, freq, resolution0); //

//   ledcAttachPin(B1_PWM, channel0); // 将通道0与引脚13连接
//   ledcAttachPin(B2_PWM, channel1); //
//   ledcAttachPin(B3_PWM, channel2); //
//   ledcAttachPin(A1_PWM, channel3); //
//   ledcAttachPin(A2_PWM, channel4); //
//   ledcAttachPin(A3_PWM, channel5); //
//   delay(1111);
// }

// //舵机角度设置//////////////////////////////////////////////////////////////////////////////////////////////////////////

// void Keypad_init()
// {
//   pinMode(k1pin, INPUT_PULLUP);
//   pinMode(k2pin, INPUT_PULLUP);
//   pinMode(k3pin, INPUT_PULLUP);
// }

// void Keypad_detection()
// {
//   boolean K1 = digitalRead(k1pin);
//   boolean K2 = digitalRead(k2pin);
//   boolean K3 = digitalRead(k3pin);
//   if (K1)
//   {
//     s1_press_time = 0;
//     s1_long_press = 0;
//     if (s1)
//     {
//       s1 = 0;
//       key1 = 1;
//     }
//   }
//   else
//   {
//     s1 = 1;
//     s1_press_time++;
//     if (s1_press_time > 50)
//     {
//       s1_press_time = 0;
//       s1 = 0;
//       s1_long_press = 1;
//     }
//   }

//   if (K2)
//   {
//     s2_press_time = 0;
//     s2_long_press = 0;
//     if (s2)
//     {
//       s2 = 0;
//       key2 = 1;
//     }
//   }
//   else
//   {
//     s2 = 1;
//     s2_press_time++;
//     if (s2_press_time > 50)
//     {
//       s2_press_time = 0;
//       s2 = 0;
//       s2_long_press = 1;
//     }
//   }

//   if (K3)
//   {
//     s3_press_time = 0;
//     s3_long_press = 0;
//     if (s3)
//     {
//       s3 = 0;
//       key3 = 1;
//     }
//   }
//   else
//   {
//     s3 = 1;
//     s3_press_time++;
//     if (s3_press_time > 50)
//     {
//       s3_press_time = 0;
//       s3 = 0;
//       s3_long_press = 1;
//     }
//   }

//   if (key1)
//   {
//     key1 = 0;
//     show_f++;
//     if (show_f > 7)
//       show_f = 7;
//   }
//   if (key2)
//   {
//     key2 = 0;
//     show_f--;
//     if (show_f < 0)
//       show_f = 0;
//   }

//   switch (show_f)
//   {
//   case 0:
//     Serial.print((KalmanY.Angle - AngleY_bias), 3);
//     Serial.print("\t");
//     Serial.print(AngleY, 3);
//     Serial.print("\t");
//     Serial.print(AngleX, 3);
//     Serial.print("\t");
//     Serial.print(KalmanZ.Rate);
//     Serial.println("\t");
//     break;
//   case 1:
//     // Serial.print(KalmanY.Angle);
//     // Serial.print("\t");
//     // Serial.print(AngleY_bias);
//     // Serial.print("\t");
//     Serial.print(KalmanY.Angle - AngleY_bias);
//     Serial.print("\t");
//     // Serial.print(KalmanX.Angle);
//     // Serial.print("\t");
//     // Serial.print(AngleX_bias);
//     // Serial.print("\t");
//     Serial.print(KalmanX.Angle - AngleX_bias);
//     Serial.println("\t");
//     if (key3 == 0)
//     {
//       key3 = 0;
//       EEPROM.writeFloat(30, AngleX);
//       EEPROM.writeFloat(40, AngleY);
//       EEPROM.commit();
//       AngleX_bias = EEPROM.readFloat(30);
//       AngleY_bias = EEPROM.readFloat(40);
//     }
//     break;
//   default:
//     break;
//   }
// }

// int Mpu6050Int()
// {
//   Wire.begin(19, 22, 400000); // SDA1,SCL1
//   Wire.setClock(400000);      // 400kHz I2C clock. Comment this line if having compilation difficulties
//   // initialize device
//   Serial.println(F("Initializing I2C devices..."));
//   mpu.initialize();
//   mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
//   mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
//   mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//   mpu.setSleepEnabled(false);

//   // verify connection
//   Serial.println(F("Testing device connections..."));
//   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//   // load and configure the DMP
//   Serial.println(F("Initializing DMP..."));
//   devStatus = mpu.dmpInitialize();

//   // supply your own gyro offsets here, scaled for min sensitivity
//   mpu.setXGyroOffset(-1);
//   mpu.setYGyroOffset(80);
//   mpu.setZGyroOffset(-58);
//   mpu.setXAccelOffset(-404);
//   mpu.setYAccelOffset(-2764);
//   mpu.setZAccelOffset(1218);

//   // make sure it worked (returns 0 if so)
//   if (devStatus == 0)
//   {
//     // Calibration Time: generate offsets and calibrate our MPU6050
//     // mpu.CalibrateAccel(6);
//     // mpu.CalibrateGyro(6);
//     Serial.println();
//     mpu.PrintActiveOffsets();
//     // turn on the DMP, now that it's ready
//     Serial.println(F("Enabling DMP..."));
//     mpu.setDMPEnabled(true);

//     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//     // set our DMP Ready flag so the main loop() function knows it's okay to use it
//     Serial.println(F("DMP ready! Waiting for first interrupt..."));
//     mpuIntStatus = mpu.getIntStatus();
//     imuReady = true;

//     // get expected DMP packet size for later comparison
//     packetSize = mpu.dmpGetFIFOPacketSize();
//   }
//   else
//   {
//     // ERROR!
//     // 1 = initial memory load failed
//     // 2 = DMP configuration updates failed
//     // (if it's going to break, usually the code will be 1)
//     Serial.print(F("DMP Initialization failed (code "));
//     Serial.print(devStatus);
//     Serial.println(F(")"));
//   }
//   delay(2000);
//   Serial.println(F("Adjusting DMP sensor fusion gain..."));
//   mpu.setMemoryBank(0);
//   mpu.setMemoryStartAddress(0x60);
//   mpu.writeMemoryByte(0);
//   mpu.writeMemoryByte(0x20);
//   mpu.writeMemoryByte(0);
//   mpu.writeMemoryByte(0);

//   return imuReady;
// }

// int hasDataIMU()
// {
//   return imuReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
// }

// void Mpu6050Read()
// {

//   // Get the Latest packet
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

//   float X = ypr[2] * 180 / M_PI;
//   float Y = ypr[1] * 180 / M_PI;
//   float Z = ypr[0] * 180 / M_PI;

//   AngleX = X;
//   AngleY = Y;
//   AngleZ = Z;

//   /*if(AngleX > 33)
//   AngleX = 33;

//   if(AngleX < (-33))
//   AngleX = -33;

//   if(AngleY > 33)
//   AngleY = 33;

//   if(AngleY < (-33))
//   AngleY = -33;
//   */

//   mpu.dmpGetGyro(&gy, fifoBuffer);
//   float y1 = (float)(-gy.y / 16.4);
//   float x1 = (float)(gy.x / 16.4);
//   float z1 = (float)(-gy.z / 16.4);

//   YKalmangGetAngle(AngleY, y1, Ts);
//   XKalmangGetAngle(AngleX, x1, Ts);
//   ZKalmangGetAngle(AngleZ, z1, Ts);
//   /////////////////////////////////////////////////////////////////

//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetAccel(&aa, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//   mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
// }

// void setup()
// {
//   Serial.begin(115200);
  
//   Keypad_init();
//   //增加 T 指令
//   // command.add('T', doTarget, "target velocity");
//   command.add('Q', onA1A, "target_angle");
//   command.add('W', onA2A, "target_angle");
//   command.add('E', onA3A, "target_angle");
//   command.add('Z', onB1A, "target_angle");
//   command.add('X', onB2A, "target_angle");
//   command.add('C', onB3A, "target_angle");

//   if (!Mpu6050Int())
//   {
//     Serial.println(F("IMU connection problem... Disabling!"));
//     return;
//   }

//   Kalman_init();
//   delay(111);
//   Servo_Initialization(); // ledc初始化程序

//   EEPROM_init();
//   now_us = micros();

//   velocity_calc_timestamp = now_us;
// }

// void loop()
// {
//   now_us = micros();
//   Ts = (now_us - velocity_calc_timestamp) * 1e-6;
//   if (Ts <= 0 || Ts > 0.5)
//     Ts = 1e-3;
//   if (Ts >= 0.01) // if (mpuInterrupt)
//   {
//     velocity_calc_timestamp = now_us;
//     if (mpuInterrupt)
//     {
//       if (hasDataIMU())
//       {
//         Mpu6050Read();
//       }
//       mpuInterrupt = false;
//     }
//   }
//   // mpuInterrupt = false;
//   Keypad_detection();
//   command.run();
//   Set_servo_Angle(A_Angle, B_Angle);
//   ledcWrite(channel0, map(B1_angle, 0, 900, B1angle_mini, B1angle_max)); // B1
//   ledcWrite(channel3, map(A1_angle, 0, 900, A1angle_mini, A1angle_max)); // A1
//   ledcWrite(channel1, map(B2_angle, 0, 900, B2angle_mini, B2angle_max)); // B2
//   ledcWrite(channel2, map(B3_angle, 0, 900, B3angle_mini, B3angle_max)); // B3
//   ledcWrite(channel4, map(A2_angle, 0, 900, A2angle_mini, A2angle_max)); // A2
//   ledcWrite(channel5, map(A3_angle, 0, 900, A3angle_mini, A3angle_max)); // A3
//    delay(10);
//   // Serial.printf("it is ok!");
// }
