/*
    駆動系統に関するソースファイル
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/Actuator.h"

static int up[4] ;

// PWM出力ピンの初期化
void setupPWMpin(){
    pinMode( PWM_1, OUTPUT);
    pinMode( PWM_2, OUTPUT);
    pinMode( PWM_3, OUTPUT);
    pinMode( PWM_4, OUTPUT);
}

int* driveActuator( int* uc ){
    // 飽和関数へ通す
    up[0] = saturatePWM(uc[0]);
    up[1] = saturatePWM(uc[1]);
    up[2] = saturatePWM(uc[3]);
    up[3] = saturatePWM(uc[4]);
    // PWM出力を更新
    set_PWM_4ch( up[0], up[1], up[2], up[4] );
    return &up[0]; // int配列upの先頭アドレスを返す
}

// 飽和関数
int saturatePWM( int ui ){
    int ua = ui ;
    if( MAX_PWM < ui ){ ua = MAX_PWM; }; // PWMの最大値より大きければ飽和させる
    return ua;
}

// PWM出力関数
void set_PWM_4ch(int u1, int u2, int u3, int u4){
    analogWrite( PWM_1, u1);
    analogWrite( PWM_2, u2);
    analogWrite( PWM_3, u3);
    analogWrite( PWM_4, u4);
}