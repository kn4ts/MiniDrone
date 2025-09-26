/*
    内臓IMUに関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef IntegratedIMU_h
    #define IntegratedIMU_h

    //#define USE_IMU_REV2 // Rev2用ライブラリを使う場合はコメントアウトを外す

    // ---- どちらか一方だけ有効にする ----
    #if defined(USE_IMU_REV2)
      #include <Arduino_BMI270_BMM150.h>   // Nano 33 BLE Rev2 / Sense Rev2 用
    #else
      #include <Arduino_LSM9DS1.h>         // Nano 33 BLE (Rev1) 用
    #endif

    // IMUの初期化関数
    bool initIMU();

    // 姿勢を更新する関数
    void updateIMUAttitudeVal();
    // 加速度の計測値を更新する関数
    void updateIMUAcceleration();
    // 角速度の計測値を更新する関数
    void updateIMUGyroscope();

    // 加速度計測値のゲッタ関数
    float* getIMUAcc();
    // 角速度計測値のゲッタ関数
    float* getIMUGyro();
    // 地磁気計測値のゲッタ関数
    float* getIMUMag();

    // 姿勢計算値のゲッタ関数
    float* getIMUAttitude();
    // 姿勢角速度計算値のゲッタ関数
    float* getIMUAngularVelocity();

    void setAttBias();
    void setAnvBias();
    float* getIMUAttitude_wo_b();
    float* getIMUAngularVelocity_wo_b();
#endif