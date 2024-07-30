/*
    内臓IMUに関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef IntegratedIMU_h
    #define IntegratedIMU_h

    #include <Arduino_LSM9DS1.h>

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
    float* getIMUAttitude();

#endif