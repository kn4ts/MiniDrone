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
static RollGain rolK = { 0, 0, 0 };
//static float rol_Kp = 0.2;
//static float rol_Ki = 0.01;
//static float rol_Kd = 0.01;
// ピッチ角度ゲイン
static PitchGain pitK = { 0, 0, 0 };
//static float pit_Kp = 0.2;
//static float pit_Ki = 0.01;
//static float pit_Kd = 0.01;


/* 制御器の内部変数 */
// 高度に関するもの
static AltVariable alt = { 0, 0, 0, 0, 0, 0 };
// ロール角度に関するもの
static RollVariable rol = { 0, 0, 0, 0, 0, 0};
// ピッチ角度に関するもの
static PitchVariable pit = { 0, 0, 0, 0, 0, 0};
// ヨー角度に関するもの
static YawVariable yaw = { 0, 0, 0, 0, 0, 0};

// 計測値のフィルタ処理用の変数
//static float alt_alpha = 0.95 ; // ローパスのゲイン
//static float alt_alpha = 0.90 ; // ローパスのゲイン
//static float alt_alpha = 0.85 ; // ローパスのゲイン
// static float alt_alpha = 0.80 ; // ローパスのゲイン -> おとなしくなった？
// static float alt_alpha = 0.75 ; // ローパスのゲイン
static LowpassFilterGain alpha = { 0.7, 0.8, 0.8, 0.8 };
//static float alt_alpha = 0.70 ; // ローパスのゲイン

// 要求制御力
static float cont_force[4] ; // 要求制御力をまとめる配列

// 制御器出力
static float uc[4]; // 制御器出力の配列

// バイアス入力
static float u_bias[4] = {10,10,15,15};

// 制御器実装用の変数
static unsigned long prevTime, currTime ; // 時刻の差分をとるための変数
static float deltaTime ; // 時刻の差分を格納する変数

// 飛行状態の把握に使う？内部変数
static int8_t status = 0 ; // ステータス

/* 関数定義 */
// 制御器の実装例
//   引数：フィードバックされる信号
//   返り値：PWM指令値の配列の先頭アドレス
float* controller_demo( float* y, float distance ){

    // 時間算出
    currTime = millis(); // 現在時刻の取得
    deltaTime = min( 0.001 * (currTime - prevTime), 0.02 ) ; // 前回からの差分時間[s]を計算，最大でも0.02[s]に制限
    prevTime = currTime; // 前回時刻を更新

    /* 信号の更新 */ 
    // 高度情報
    alt.filt = lowpassFilterAltitude_demo( alt.filt_prev, distance ) ; // 高度計測値にローパスフィルタをかける
    //
    alt.ed = ( -alt.filt +alt.filt_prev ) / deltaTime ; // 微分先行で計測値を数値微分
    alt.e = ref_alt -alt.filt ; // 現在の誤差
    alt.ei += alt.e * deltaTime ; // 誤差の積分
    //
    alt.filt_prev = alt.filt ; // 1ステップ前のフィルタ後高度を更新
    //
    // ロール角度情報
    rol.filt = lowpassFilterRoll_demo( rol.filt_prev, y[0] ) ; // ロール角計測値にローパスフィルタをかける
    //
    //rol.e = ref_rol -y[0] ; // 現在の誤差
    rol.e = ref_rol -rol.filt ; // 現在の誤差
    rol.ed = ( rol.e -rol.filt_prev ) / deltaTime ; // 誤差を数値微分
    rol.ei += rol.e * deltaTime ; // 誤差の積分
    //
    rol.prev = rol.e ; // 1ステップ前の誤差を更新
    //
    // ピッチ角度情報
    pit.filt = lowpassFilterPitch_demo( pit.filt_prev, y[1] ) ; // ピッチ角計測値にローパスフィルタをかける
    //
    //pit.e = ref_pit -y[1] ; // 誤差
    pit.e = ref_pit -pit.filt ; // 誤差
    pit.ed = ( pit.e -pit.filt_prev ) / deltaTime ; // 誤差の数値微分
    pit.ei += pit.e * deltaTime ; // 誤差の積分
    //
    pit.prev = pit.e ; // 1ステップ前の誤差を更新
    //
    // ヨー角度情報
    yaw.filt = lowpassFilterYaw_demo( yaw.filt_prev, y[2] ) ; // ヨー角計測値にローパスフィルタをかける
    //
    //pit.e = ref_pit -y[1] ; // 誤差
    yaw.e = ref_pit -y[2] ; // 誤差
    yaw.ed = ( yaw.e -yaw.prev ) / deltaTime ; // 誤差を数値微分
    yaw.ei += yaw.e * deltaTime ; // 誤差の積分
    //
    yaw.prev = yaw.e ; // 1ステップ前の誤差を更新

    // ステータス（飛行状況）の判定・更新
    if( status == 0 && distance > 6 ){
        status = 1; // 離陸
    }

    // 要求制御力の計算
    float tau_rol = rolK.p * rol.e + rolK.i * rol.ei + rolK.d * rol.ed ; // ロール方向
    float tau_pit = pitK.p * pit.e + pitK.i * pit.ei + pitK.d * pit.ed ; // ピッチ方向
    float tau_yaw = 0.0 ;
    float f_total = altK.p * alt.e + altK.i * alt.ei + altK.d * alt.ed ; // 高度方向

    // ミキシング（分配）
    allocator_demo( tau_rol, tau_pit, tau_yaw, f_total );

    // 変数保存&更新
    cont_force[0] = tau_rol;
    cont_force[1] = tau_pit;
    cont_force[2] = tau_yaw;
    cont_force[3] = f_total;

    // 計算した制御入力（PWM値）の配列の先頭アドレスを返す
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
float lowpassFilterAltitude_demo( float alt_filt_prev, float dist ){
    // 高度計測値に1次のローパスフィルタをかける
    float alt_filt_new = ( 1 - alpha.alt ) * alt_filt_prev + alpha.alt * dist ;
    return alt_filt_new;
}
float lowpassFilterRoll_demo( float rol_filt_prev, float rol ){
    // ロール角計測値に1次のローパスフィルタをかける
    float rol_filt_new = ( 1 - alpha.rol ) * rol_filt_prev + alpha.rol * rol ;
    return rol_filt_new;
}
float lowpassFilterPitch_demo( float pit_filt_prev, float pit ){
    // ピッチ角計測値に1次のローパスフィルタをかける
    float pit_filt_new = ( 1 - alpha.pit ) * pit_filt_prev + alpha.pit * pit ;
    return pit_filt_new;
}
float lowpassFilterYaw_demo( float yaw_filt_prev, float yaw ){
    // 高度計測値に1次のローパスフィルタをかける
    float yaw_filt_new = ( 1 - alpha.yaw ) * yaw_filt_prev + alpha.yaw * yaw ;
    return yaw_filt_new;
}

// フィルタ処理後の値のゲッタ関数
float getAltitudeFiltered(){ return alt.filt; } // 高度について
float getRollFiltered(){ return rol.filt; } // ロール角について
float getPitchFiltered(){ return pit.filt; } // ピッチ角について
float getYawFiltered(){ return yaw.filt; } // ピッチ角について
// 要求制御力のゲッタ関数
float* getControlForceReq(){ return &cont_force[0]; } // 

// ロール角度指令値のセッタ関数
void setRollReference( float r ){ ref_rol = r; } // ピッチ角度指令値を更新
// ピッチ角度指令値のセッタ関数
void setPitchReference( float r ){ ref_pit = r; } // ピッチ角度指令値を更新

// 制御器のリセット（状態変数の初期化）関数
void initializeController(){
    // 制御器の内部変数のリセット
    alt = {0, 0, 0, 0, 0, 0};
    rol = {0, 0, 0, 0, 0, 0};
    pit = {0, 0, 0, 0, 0, 0};
    yaw = {0, 0, 0, 0, 0, 0};
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