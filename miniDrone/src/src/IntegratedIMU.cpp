/*
    内臓IMUに関するソースファイル
      Arduino nano 33 BLE に内臓のIMU（LSM9DS1）を使用する
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/IntegratedIMU.h"

// センサ計測値（生データ）を格納する変数
static float imu_ac[3] ;   // 加速度計測値 x, y, z 方向
static float imu_gy[3] ;   // ジャイロ計測値 x, y, z 軸周り
static float imu_mg[3] ;   // 地磁気計測値 x, y, z 方向

// センサ計測値のバイアスを格納する変数
static float imu_gy_bias[3] = {0,0,0} ; // ジャイロ計測値のバイアス x, y, z 軸周り

// 計算した物理量を格納する変数
static float attitude[3] ;  // 姿勢角 roll, pitch, yaw
static float angularvelocity[3]; // 姿勢角速度 roll, pitch, yaw

// 物理量補正のための変数
static float att_bias[3] ; // 姿勢角のバイアス
static float anv_bias[3] ; // 姿勢角速度のバイアス

// 物理量補正のための変数
static float att[3] ; // 姿勢角（バイアス補正後）
static float anv[3] ; // 姿勢角速度（バイアス補正後）

// 相補フィルタ計算のための変数定義
static unsigned long prevTime, currTime ; // 時刻の差分をとるための変数（ミリ秒）
static float deltaTime ; // 時刻の差分を格納する変数（秒）

static unsigned long prevTime_us, currTime_us ; // 時刻の差分をとるための変数（マイクロ秒）

// 内臓IMUの初期化関数
bool initIMU(){
    bool state = true; // 戻り値の初期化

    // バイアス値を0に初期化
    att_bias[0] = 0;
    att_bias[1] = 0;
    att_bias[2] = 0;
    anv_bias[0] = 0;
    anv_bias[1] = 0;
    anv_bias[2] = 0;

    if (!IMU.begin()){ state = false; } // 初期化に失敗すると戻り値をfalseに設定

    // 初回計測でジャイロセンサのバイアスを取得
    while(!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()){
      // 加速度計とジャイロが利用可能になるまで待つ
      delay(10);
    }
    IMU.readGyroscope(imu_gy_bias[0], imu_gy_bias[1], imu_gy_bias[2]); // ジャイロ計測値を取得

    return state;
}

// 相補フィルタを使った姿勢推定
void updateIMUAttitudeVal(){

    static const float alpha = 0.98; // 相補フィルタの係数
    //static const float alpha = 0.95; // 相補フィルタの係数
    static const float alpha_mag = 0.95; // 相補フィルタの係数（ヨー方向）

    // 加速度計測値とジャイロ計測値が利用可能になっていれば姿勢推定値を更新
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

        currTime = millis(); // 現在時刻の取得
        deltaTime = min( 0.001 * (currTime - prevTime), 0.02) ; // 前回からの差分時間を計算
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
            //float magX = imu_mg[0] * cos(attitude[1]) + imu_mg[2] * sin(attitude[1]);
            //float magY = imu_mg[0] * sin(attitude[0]) * sin(attitude[1]) + imu_mg[1] * cos(attitude[0]) - imu_mg[2] * sin(attitude[0]) * cos(attitude[1]);
            //float yawMag = atan2(magY, magX) * 180.0 / PI;
            float yawMag = atan2(imu_mg[1], imu_mg[0]) * 180.0 / PI;
            if(yawMag < -180){ yawMag += 360; } // -180度を下回っていたら+360度で補正

            // 相補フィルタでジャイロと地磁気の計測値を統合
            //attitude[2] = alpha * attitude[2] + (1.0 - alpha) * yawMag;
            float att_yaw_new = alpha_mag * attitude[2] + (1.0 - alpha_mag) * yawMag;
            // ヨー方向角速度の計算（数値微分）
            angularvelocity[2] = ( att_yaw_new - attitude[2] ) / deltaTime ;
            // ヨー方向姿勢角の更新
            attitude[2] = att_yaw_new ;
        }

        // 加速度センサ計測値から姿勢を計算
        float rollAcc = atan2(imu_ac[1], imu_ac[2]) * 180.0 / PI;
        float pitchAcc = atan2(-imu_ac[0], sqrt(imu_ac[1] * imu_ac[1] + imu_ac[2] * imu_ac[2])) * 180.0 / PI;

        // 相補フィルタでジャイロと加速度から求めた姿勢角を統合
        float att_roll_new  = alpha * attitude[0] + (1.0 - alpha) * rollAcc;
        float att_pitch_new = alpha * attitude[1] + (1.0 - alpha) * pitchAcc;

        // 角速度の計算（数値微分）
        angularvelocity[0] = (att_roll_new  - attitude[0]) / deltaTime ;
        angularvelocity[1] = (att_pitch_new - attitude[1]) / deltaTime ;

        // 姿勢角の更新
        attitude[0] = att_roll_new ;
        attitude[1] = att_pitch_new ;
    }
}

//
void updateIMUAttitudeVal_ver2(){

    // センサ準備（できていないなら姿勢更新なし）
    if (!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) return;

    // 時間計測（us精度）とクリップ
    currTime_us = micros();
    deltaTime = (currTime_us - prevTime_us) * 1.0e-6f;
    if (deltaTime < 0.0005f) deltaTime = 0.0005f;  // 下限
    if (deltaTime > 0.0200f) deltaTime = 0.0200f;  // 上限（50 Hz 相当）
    prevTime_us = currTime_us;

    //static unsigned long prev_us = micros(); // 初回のみ時刻代入
    //unsigned long now_us = micros();
    //if (deltaTime < 0.0005f) deltaTime = 0.0005f;  // 下限
    //if (deltaTime > 0.0200f) deltaTime = 0.0200f;  // 上限（50 Hz 相当）

    // センサ読み出し
    IMU.readAcceleration(imu_ac[0], imu_ac[1], imu_ac[2]);   // 単位[g]
    IMU.readGyroscope(imu_gy[0], imu_gy[1], imu_gy[2]);      // 単位[deg/s]

    // ジャイロ・バイアス補正（起動時に imu_gy_bias に計測値をセットしておく）
    float gx = imu_gy[0] - imu_gy_bias[0];
    float gy = imu_gy[1] - imu_gy_bias[1];
    float gz = imu_gy[2] - imu_gy_bias[2];

    // ジャイロ積分（予測）
    attitude[0] += gx * deltaTime;   // roll[deg]
    attitude[1] += gy * deltaTime;   // pitch[deg]
    attitude[2] += gz * deltaTime;   // yaw[deg]

    // 相補フィルタ係数（時定数τから都度算出）
    const float tau_att = 0.7f;   // roll/pitch 用（0.5–1.0 s 推奨）
    const float tau_yaw = 2.0f;   // yaw 用（1–3 s 推奨）
    float a_att = tau_att / (tau_att + deltaTime ); // roll/pitch 用
    float a_yaw = tau_yaw / (tau_yaw + deltaTime ); // yaw 用

    // 加速度からの姿勢（観測）※右手系：x前, y右, z下基準の一般式
    float rollAcc  = atan2(imu_ac[1], imu_ac[2]) * RAD_TO_DEG;
    float pitchAcc = atan2(-imu_ac[0],
                           sqrtf(imu_ac[1]*imu_ac[1] + imu_ac[2]*imu_ac[2])) * RAD_TO_DEG;
    
    // 相補（roll/pitch）
    attitude[0] = a_att * attitude[0] + (1.0f - a_att) * rollAcc;
    attitude[1] = a_att * attitude[1] + (1.0f - a_att) * pitchAcc;

    // ヨー：地磁気があれば傾斜補償で観測 → 相補
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(imu_mg[0], imu_mg[1], imu_mg[2]); // 単位[µT]

        // 現在の roll/pitch で水平面に補償
        float cr = cosf(attitude[0] * DEG_TO_RAD), sr = sinf(attitude[0] * DEG_TO_RAD);
        float cp = cosf(attitude[1] * DEG_TO_RAD), sp = sinf(attitude[1] * DEG_TO_RAD);
        // 水平面へ投影
        float mxh = imu_mg[0]*cp + imu_mg[2]*sp;
        float myh = imu_mg[0]*sr*sp + imu_mg[1]*cr - imu_mg[2]*sr*cp;

        float yawMag = atan2f(myh, mxh) * RAD_TO_DEG;  // [-180,180]へ正規化
        if (yawMag >  180.0f) yawMag -= 360.0f;
        if (yawMag < -180.0f) yawMag += 360.0f;

        attitude[2] = a_yaw * attitude[2] + (1.0f - a_yaw) * yawMag;
    }
    // ※地磁気が無い周期はジャイロ積分のみ（上の予測値をそのまま）

    // 角速度は「数値微分」ではなくジャイロ値をそのまま使う（制御のD項向け）
    angularvelocity[0] = gx;  // [deg/s]
    angularvelocity[1] = gy;  // [deg/s]
    angularvelocity[2] = gz;  // [deg/s]

    // 姿勢オフセット（水平合わせなど）※必要に応じて
    //attitude[0] -= att_bias[0];
    //attitude[1] -= att_bias[1];
    //attitude[2] -= att_bias[2];
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
// 姿勢角の現在値をバイアス値にセットする関数
void setAttBias(){
    att_bias[0] = attitude[0];
    att_bias[1] = attitude[1];
    att_bias[2] = attitude[2];
}
// 姿勢角速度の現在値をバイアス値にセットする関数
void setAnvBias(){
    anv_bias[0] = angularvelocity[0];
    anv_bias[1] = angularvelocity[1];
    anv_bias[2] = angularvelocity[2];
}
// バイアス処理後の姿勢角のゲッタ
float* getIMUAttitude_wo_b(){
    att[0] = attitude[0] -att_bias[0];
    att[1] = attitude[1] -att_bias[1];
    att[2] = attitude[2] -att_bias[2];
    return &att[0];
}
// バイアス処理後の姿勢角速度のゲッタ
float* getIMUAngularVelocity_wo_b(){
    anv[0] = angularvelocity[0] -anv_bias[0];
    anv[1] = angularvelocity[1] -anv_bias[1];
    anv[2] = angularvelocity[2] -anv_bias[2];
    return &anv[0];
}

// 加速度計測値のゲッタ関数
float* getIMUAcc(){ return &imu_ac[0];}
// 角速度計測値のゲッタ関数
float* getIMUGyro(){ return &imu_gy[0];}
// 地磁気計測値のゲッタ関数
float* getIMUMag(){ return &imu_mg[0];}

// 姿勢角のゲッタ関数
float* getIMUAttitude(){ return &attitude[0];}
// 姿勢角速度のゲッタ関数
float* getIMUAngularVelocity(){ return &angularvelocity[0];}