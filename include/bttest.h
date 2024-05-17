#ifndef __BNTTEST_H__
#define __BNTTEST_H__

#pragma once
#include <BluetoothSerial.h>

void BT_init();

char BT_testrun();

#endif

BluetoothSerial SerialBT;

void BT_init(){

  SerialBT.begin("test2233");


}


char BT_testrun(){
    char temp = '0';
    if(SerialBT.available()){
        temp = SerialBT.read();
    }
    return temp;
}

void BT_back(float a){
    SerialBT.println(a);
}