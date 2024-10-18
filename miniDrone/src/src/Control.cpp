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
static float ref_alt = 10;  // 高度目標値[mm]
static float ref_rol = 0;   // ロール目標値[degree?]
static float ref_pit = 0;   // ピッチ目標値[degree?]
static float ref_yaw = 0;   // ヨー目標値[degree?]

/* 制御器ゲイン */
// 高度ゲイン
// 一番まし？
//static float alt_Kp = 2;
//static float alt_Ki = 2;
//static float alt_Kd = 0.003;
static AltGain altK = { 2, 1, 0.001 };
//static float alt_Kp = 2;
//static float alt_Ki = 2;
//static float alt_d = 0.05;
//static float alt_d = 0.1;
//static float alt_d = 0.01;
//static float alt_Kd = 0.003;
// ロール角度ゲイン
static float rol_Kp = 0;
static float rol_Ki = 0;
static float rol_Kd = 0;
//static float rol_Kp = 0.2;
//static float rol_Ki = 0.01;
//static float rol_Kd = 0.01;
// ピッチ角度ゲイン
static float pit_Kp = 0;
static float pit_Ki = 0;
static float pit_Kd = 0;
//static float pit_Kp = 0.2;
//static float pit_Ki = 0.01;
//static float pit_Kd = 0.01;


/* 制御器の状態変数 */
// 高度に関するもの
static AltVariable alt = { 0, 0, 0, 0, 0, 0 };
//static float alt_e = 0;     // 現在の誤差
//static float alt_ei = 0;    // 誤差の積分
//static float alt_ed = 0;    // 誤差の微分
//static float prev_alt = 0;  // 1ステップ前の高度
// ロール角度に関するもの
static float rol_e = 0;
static float rol_ei = 0;
static float rol_ed = 0;
static float prev_rol_e = 0; // 1ステップ前のロール角誤差
// ピッチ角度に関するもの
static float pit_e = 0;
static float pit_ei = 0;
static float pit_ed = 0;
static float prev_pit_e = 0;
// ヨー角度に関するもの
static float yaw_e = 0;
static float yaw_ei = 0;
static float yaw_ed = 0;
static float prev_yaw_e = 0;

// 計測値のフィルタ処理用の変数
static float alt_filtered = 0;
static float prev_alt_filtered = 0;
static float alt_alpha = 0.95 ;

// 要求制御力
static float cont_force[4] ; // 要求制御力をまとめる配列

// 制御器出力
static float uc[4]; // 制御器出力の配列

// バイアス入力
static float u_bias[4] = {10,10,15,15};

// 制御器実装用の変数
static unsigned long prevTime, currTime ; // 時刻の差分をとるための変数
static float deltaTime ; // 時刻の差分を格納する変数

static int8_t status = 0 ; // ステータス

/* 関数定義 */
// 制御器の実装例
//   引数：フィードバックされる信号
//   返り値：PWM指令値の配列の先頭アドレス
float* controller_demo( float* y, uint16_t distance ){

    // 時間算出
    currTime = millis(); // 現在時刻の取得
    deltaTime = min( 0.001 * (currTime - prevTime), 0.02 ) ; // 前回からの差分時間[s]を計算，最大でも0.02[s]に制限
    prevTime = currTime; // 前回時刻を更新

    /* 信号の更新 */ 
    // 高度情報
    alt.filt = lowpassFilterAltitude_demo( distance, alt.filt_prev ) ; // 高度計測値にローパスフィルタをかける
    //
    alt.ed = ( -alt.filt +alt.filt_prev ) / deltaTime ; // 微分先行で計測値を数値微分
    alt.e = ref_alt -alt.filt ; // 現在の誤差
    alt.ei += alt.e * deltaTime ; // 誤差の積分
    //
    alt.filt_prev = alt.filt ; // 1ステップ前のフィルタ後高度を更新
    /*alt_ed = ( -(float)alt +prev_alt ) / deltaTime ; // 微分先行で計測値を数値微分
    alt_e = ref_alt -(float)alt ; // 現在の誤差
    alt_ei += alt_e * deltaTime ; // 誤差の積分*/
    //prev_alt = alt ; // 1ステップ前の高度を更新
    // ロール角度情報
    rol_e = ref_rol -y[0] ; // 現在の誤差
    rol_ed = ( rol_e -prev_rol_e ) / deltaTime ; // 誤差を数値微分
    rol_ei += rol_e * deltaTime ; // 誤差の積分
    prev_rol_e = rol_e ; // 1ステップ前の誤差を更新
    // ピッチ角度情報
    pit_e = ref_pit -y[1] ; // 誤差
    pit_ed = ( pit_e -prev_pit_e ) / deltaTime ; // 微分先行で計測値を数値微分
    pit_ei += pit_e * deltaTime ; // 誤差の積分
    prev_pit_e = pit_e ; // 1ステップ前の誤差を更新

    // ステータス（飛行状況）の判定・更新
    if( status == 0 && distance > 6 ){
        status = 1; // 離陸
    }

    // 要求制御力の計算
    float tau_rol = rol_Kp * rol_e + rol_Ki * rol_ei + rol_Kd * rol_ed ; // ロール方向
    float tau_pit = pit_Kp * pit_e + pit_Ki * pit_ei + pit_Kd * pit_ed ; // ピッチ方向
    float tau_yaw = 0.0 ;
    float f_total = altK.p * alt.e + altK.i * alt.ei + altK.d * alt.ed ; // 高度方向

    // ミキシング（分配）
    allocator_demo( tau_rol, tau_pit, tau_yaw, f_total );

    // 変数保存&更新
    cont_force[0] = tau_rol;
    cont_force[1] = tau_pit;
    cont_force[2] = tau_yaw;
    cont_force[3] = f_total;

    return &uc[0];
}

// 分配器の実装例
void allocator_demo( float t_r, float t_p, float t_y, float f_t ){
    uc[0] = (-1) * t_r + (+1) * t_p + (+1) * t_y + (+1) * f_t + u_bias[0] ;
    uc[1] = (-1) * t_r + (-1) * t_p + (-1) * t_y + (+1) * f_t + u_bias[1] ;
    uc[2] = (+1) * t_r + (-1) * t_p + (+1) * t_y + (+1) * f_t + u_bias[2] ;
    uc[3] = (+1) * t_r + (+1) * t_p + (-1) * t_y + (+1) * f_t + u_bias[3] ;
}

// ローパスフィルタの実装例
float lowpassFilterAltitude_demo( uint16_t dist, float alt_filt_prev ){
    // 高度計測値に1次のローパスフィルタをかける
    float alt_filt_new = alt_alpha * (float)dist + ( 1 - alt_alpha ) * alt_filt_prev ;
    return alt_filt_new;
}

// フィルタ処理後の高度のゲッタ関数
float getAltitudeFiltered(){ return alt.filt; }
// 要求制御力のゲッタ関数
float* getControlForceReq(){ return &cont_force[0]; } // 

// ロール角度指令値のセッタ関数
void setRollReference( float r ){ ref_rol = r; } // ピッチ角度指令値を更新
// ピッチ角度指令値のセッタ関数
void setPitchReference( float r ){ ref_pit = r; } // ピッチ角度指令値を更新

// 制御器のリセット（状態変数の初期化）関数
void initializeController(){
    // 積分器のリセット
    alt.ei = 0;
    rol_ei = 0;
    pit_ei = 0;
    // 状態変数のリセット
    alt.prev = 0 ;
    prev_rol_e = 0 ;
    prev_pit_e = 0 ;
    // 目標値のリセット
    ref_rol = 0;
    ref_pit = 0;
    ref_yaw = 0;
    // 制御器出力のリセット
    uc[0] = 0;
    uc[1] = 0;
    uc[2] = 0;
    uc[3] = 0;
}