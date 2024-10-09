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
static float alt_p = 2;
static float alt_i = 0.02;
static float alt_d = 1;

// 制御器の状態
static float alt_ei = 0;

static float uc[4]; // 制御器出力の配列

float* controller_demo( float* y, uint16_t alt ){

    // 信号の更新
    alt_ei += ( ref_alt -(float)alt ) ; // 誤差を積分

    // 制御器の計算
    float tau_roll = 0.0 ;
    float tau_pitch = 0.0 ;
    float tau_yaw = 0.0 ;
    float f_total = alt_p * ( ref_alt -(float)alt ) + alt_i * alt_ei ;

    // ミキシング（分配）
    allocator_demo( tau_roll, tau_pitch, tau_yaw, f_total );

    return &uc[0];
}

void allocator_demo( float t_r, float t_p, float t_y, float f_t ){
    uc[0] = (-1) * t_r + (+1) * t_p + (+1) * t_y + (+1) * f_t ;
    uc[1] = (-1) * t_r + (-1) * t_p + (-1) * t_y + (+1) * f_t ;
    uc[2] = (+1) * t_r + (-1) * t_p + (+1) * t_y + (+1) * f_t ;
    uc[3] = (+1) * t_r + (+1) * t_p + (-1) * t_y + (+1) * f_t ;
}

void initializeController(){
    alt_ei = 0;
    uc[0] = 0;
    uc[1] = 0;
    uc[2] = 0;
    uc[3] = 0;
}