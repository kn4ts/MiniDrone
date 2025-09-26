/*
    内臓IMUに関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef IntegratedIMU_h
    #define IntegratedIMU_h

    #include <Arduino_LSM9DS1.h>

    #define RAD_TO_DEG 57.2957795131f // ラジアン→度変換係数
    #define DEG_TO_RAD 0.01745329251f // 度→ラジアン変換係数

    // IMUの初期化関数
    bool initIMU();

    // 姿勢を更新する関数
    void updateIMUAttitudeVal();
    void updateIMUAttitudeVal_ver2();
    // 加速度の計測値を更新する関数
    void updateIMUAcceleration();
    // 角速度の計測値を更新する関数
    void updateIMUGyroscope();
    // ジャイロセンサのバイアスをキャリブレーションする関数
    void calibrateGyroBias();

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