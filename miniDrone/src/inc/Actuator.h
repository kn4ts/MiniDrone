/*
    駆動系に関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef Actuator_h // 複数回読み込みを防ぐif def文
    #define Actuator_h

    #include<Arduino.h>

    /* ピン定義
        4   1
         \ /
          ↑ head
         / \
        3   2

        番号    |  1  |  2  |  3  |  4  |
        回転方向 | CCW | CW | CCW  | CW |
        ピン配置 | D5 | D6  | D7  | D8  |
    */
    #define PWM_1 D5
    #define PWM_2 D6
    #define PWM_3 D7
    #define PWM_4 D8

    // アクチュエータの調整パラメータ
    // #define MIN_PWM_CW 70 // CW方向回転のPWMの最小値
    // #define MIN_PWM_CCW 64    // CCW方向回転のPWMの最小値

    //#define MAX_PWM 30 // PWM の最大値（※注意※単一モータで過大なPWM値を使用するとFETが燃える？）
    #define MAX_PWM 100 // PWM の最大値（※注意※単一モータで過大なPWM値を使用するとFETが燃える？）→100くらいなら大丈夫そう

    // 関数のプロトタイプ宣言
    void setupPWMpin();   // PWMピンの初期化関数

    int* driveActuator( int* uc );

    int saturatePWM(int ui);
    void set_PWM_4ch(int u1, int u2, int u3, int u4);   // 4chのPWM出力設定
    // uint16_t getAltitudeVal();  // 高度計測値のゲッタ関数

#endif  // if def文の終わり