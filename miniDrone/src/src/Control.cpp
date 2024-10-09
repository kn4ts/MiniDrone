/*
    制御系に関するソースファイル
     姿勢制御，高度制御など
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/Control.h"

// 目標値
// static float ref_alt = 0;
// static float ref_alt = 20;
//static float ref_alt = 10;
static float ref_alt = 10;

// 制御器ゲイン
static float alt_p = 1;
static float alt_i = 1;
static float alt_d = 0.01;

// 制御器の状態
static float alt_e = 0;
static float alt_ei = 0;
static float alt_ed = 0;
static float prev_alt = 0;
static float uc[4]; // 制御器出力の配列

// バイアス入力
static float u_bias = 10;

// 制御器実装用の変数
static unsigned long prevTime, currTime ; // 時刻の差分をとるための変数
static float deltaTime ; // 時刻の差分を格納する変数

/* 関数定義 */
// 制御器の実装例
float* controller_demo( float* y, uint16_t alt ){

    // 時間算出
    currTime = millis(); // 現在時刻の取得
    deltaTime = min( 0.001 * (currTime - prevTime), 0.02 ) ; // 前回からの差分時間[s]を計算
    prevTime = currTime; // 前回時刻を更新

    // 信号の更新
    alt_ed = ( -(float)alt +prev_alt ) / deltaTime ; // 微分先行で計測値を数値微分
    alt_e = ref_alt -(float)alt ;
    alt_ei += alt_e * deltaTime ; // 誤差を積分
    prev_alt = alt ;

    // 制御力の計算
    float tau_roll = 0.0 ;
    float tau_pitch = 0.0 ;
    float tau_yaw = 0.0 ;
    float f_total = alt_p * alt_e + alt_i * alt_ei + alt_d * alt_ed ;

    // ミキシング（分配）
    allocator_demo( tau_roll, tau_pitch, tau_yaw, f_total );

    return &uc[0];
}

// 分配器の実装例
void allocator_demo( float t_r, float t_p, float t_y, float f_t ){
    uc[0] = (-1) * t_r + (+1) * t_p + (+1) * t_y + (+1) * f_t + u_bias ;
    uc[1] = (-1) * t_r + (-1) * t_p + (-1) * t_y + (+1) * f_t + u_bias ;
    uc[2] = (+1) * t_r + (-1) * t_p + (+1) * t_y + (+1) * f_t + u_bias ;
    uc[3] = (+1) * t_r + (+1) * t_p + (-1) * t_y + (+1) * f_t + u_bias ;
}

// 制御器のリセット（状態変数の初期化）関数
void initializeController(){
    alt_ei = 0;
    prev_alt = 0 ;
    uc[0] = 0;
    uc[1] = 0;
    uc[2] = 0;
    uc[3] = 0;
}