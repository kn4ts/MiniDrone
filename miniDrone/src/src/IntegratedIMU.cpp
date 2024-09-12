/*
    内臓IMUに関するソースファイル
      Arduino nano 33 BLE に内臓のIMU（LSM9DS1）を使用する
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/IntegratedIMU.h"

static float imu_ac[3] ;   // 加速度 x, y, z 方向
static float imu_gy[3] ;   // ジャイロ計測値 x, y, z 軸周り
static float imu_mg[3] ;   // 地磁気計測値 x, y, z 方向

static float attitude[3] ;  // 姿勢角 roll, pitch, yaw

// 相補フィルタ計算のための変数定義
static unsigned long prevTime, currTime ; // 時刻の差分をとるための変数
static float deltaTime ; // 時刻の差分を格納する変数

static const float alpha = 0.98; // 相補フィルタの係数

// 内臓IMUの初期化関数
bool initIMU(){
    bool state = true;
    if (!IMU.begin()){
        state = false;
    }
    return state;
}

// 相補フィルタを使った姿勢推定
void updateIMUAttitudeVal(){

    // 加速度計測値とジャイロ計測値が利用可能になっていれば姿勢推定値を更新
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

        currTime = millis(); // 現在時刻の取得
        deltaTime = 0.001 * (currTime - prevTime) ; // 前回からの差分時間を計算
        prevTime = currTime; // 前回時刻を更新

        IMU.readAcceleration(imu_ac[0], imu_ac[1], imu_ac[2]); // 加速度計測値を取得
        IMU.readGyroscope(imu_gy[0], imu_gy[1], imu_gy[2]); // ジャイロ計測値を取得
    
        // ジャイロ計測値を積分して姿勢角を更新
        attitude[0] += imu_gy[0] * deltaTime; // ロール方向の姿勢角度を更新
        attitude[1] += imu_gy[1] * deltaTime; // ピッチ方向の姿勢角度を更新
        attitude[2] += imu_gy[2] * deltaTime; // ヨー方向の姿勢角度を更新
    
        // 地磁気センサが使用できるときは，yaw角を相補フィルタで更新
        if(IMU.magneticFieldAvailable()){
            // 地磁気計測値を取得
            IMU.readMagneticField(imu_mg[0],imu_mg[1],imu_mg[2]);
            // 地磁気センサの計測値からヨー角を計算
            float magX = imu_mg[0] * cos(attitude[1]) + imu_mg[2] * sin(attitude[1]);
            float magY = imu_mg[0] * sin(attitude[0]) * sin(attitude[1]) + imu_mg[1] * cos(attitude[0]) - imu_mg[2] * sin(attitude[0]) * cos(attitude[1]);
            float yawMag = atan2(-magY, magX) * 180.0 / PI;

            // 相補フィルタでジャイロと地磁気の計測値を統合
            attitude[2] = alpha * attitude[2] + (1.0 - alpha) * yawMag;
        }

        // 加速度センサ計測値から姿勢を計算
        float rollAcc = atan2(imu_ac[1], imu_ac[2]) * 180.0 / PI;
        float pitchAcc = atan2(-imu_ac[0], sqrt(imu_ac[1] * imu_ac[1] + imu_ac[2] * imu_ac[2])) * 180.0 / PI;

        // 相補フィルタでジャイロと加速度から求めた姿勢角を統合
        attitude[0] = alpha * attitude[0] + (1.0 - alpha) * rollAcc;
        attitude[1] = alpha * attitude[1] + (1.0 - alpha) * pitchAcc;
    }
}

// 加速度センサの計測値が利用可能であれば取得する関数
void updateIMUAcceleration(){
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(imu_ac[0], imu_ac[1], imu_ac[2]);
    }
}

// ジャイロセンサの計測値が利用可能であれば取得する関数
void updateIMUGyroscope(){
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(imu_gy[0], imu_gy[1], imu_gy[2]);
    }
}

// 加速度計測値のゲッタ関数
float* getIMUAcc(){
    return &imu_ac[0];
}

// 姿勢角のゲッタ関数
float* getIMUAttitude(){
    return &attitude[0];
}