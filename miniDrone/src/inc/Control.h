/*
    制御系に関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef Control_h // 複数回読み込みを防ぐif def文
    #define Control_h

    #include <Arduino.h>
    
    float* controller_demo(float* y, uint16_t alt) ;
    void allocator_demo(float t_r, float t_p, float t_y, float f_t) ;

    void setRollReference(float ref) ;
    void setPitchReference(float ref) ;

    float* getControlForceReq() ;

    void initializeController() ;
#endif