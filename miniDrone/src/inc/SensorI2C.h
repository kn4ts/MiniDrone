/*
    I2C接続のセンサに関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef SensorI2C_h // 複数回読み込みを防ぐif def文
    #define SensorI2C_h

    #include <Wire.h>    // I2C通信のためのライブラリ
    #include <VL53L0X.h> // ToF測距センサのライブラリ

    bool initSensorI2C();   // I2Cセンサの初期化関数
    void updateAltitudeVal();   // 高度情報の更新
    uint16_t getAltitudeVal();  // 高度計測値のゲッタ関数

#endif  // if def文の終わり