/*
    シリアル通信に関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef Serial_h
    #define Serial_h

    #include<Arduino.h>

    #define MyBaudRate 115200   // シリアル通信のボーレート

    // シリアル通信を始める関数
    void openSerial();
    // "Hello"というメッセージを送る関数
    void printHello();
    // "No central"というメッセージを送る関数
    void printNoCentral();

#endif