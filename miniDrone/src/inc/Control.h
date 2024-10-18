/*
    制御系に関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef Control_h // 複数回読み込みを防ぐif def文
    #define Control_h

    #include <Arduino.h>
    
    struct AltGain{ // 高度制御器ゲインの構造体の定義
        float p, // Pゲイン
              i, // Iゲイン
              d; // Dゲイン
    };
    struct AltVariable{ // 高度制御器の内部変数の構造体の定義
        float e,    // 誤差
              ei,   // 誤差の積分
              ed,   // 誤差の微分
              prev, // 1ステップ前の信号
              filt, // フィルタ処理後の信号
              filt_prev; // フィルタ処理後かつ1ステップ前の信号
    }; 

    // 制御器の実装例
    float* controller_demo(float* y, uint16_t distance);
    // ミキシング則（分配器）の実装例
    void allocator_demo(float t_r, float t_p, float t_y, float f_t);
    // 高度計測値のローパス処理の実装例
    float lowpassFilterAltitude_demo( uint16_t dist, float alt_filt_prev );

    void setRollReference(float ref) ; // ロール角指令値のセッタ
    void setPitchReference(float ref) ; // ピッチ角指令値のセッタ

    float getAltitudeFiltered() ; // フィルタ処理後の高度のゲッタ
    float* getControlForceReq() ; // 要求制御力のゲッタ

    void initializeController() ; // 制御器の初期化関数
#endif