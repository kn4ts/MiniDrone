/*
    シリアル通信に関するソースファイル
     有線接続なので基本的にはデバッグ用
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/Serial.h"

void openSerial(){
    Serial.begin(MyBaudRate);
    Serial.println("Connected!");
}

void printHello(){
    Serial.println("Hello");
}

void printNoCentral(String str){
    Serial.print("No central device, Bluetooth MAC:");
    Serial.println(str);
}