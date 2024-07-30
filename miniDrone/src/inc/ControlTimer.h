/*
    時間管理のためのタイマに関するヘッダファイル
      mbed OS のTickerクラスを使用する
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef ControlTimer_h
    #define ControlTimer_h

    // Ticker を使うためのライブラリ導入
    #include <mbed.h>

    // タイマーのセットアップ関数
    void setupTimer();
    // タイマー周期毎に実行する関数
    void onTimerCon();
    void onTimerBle();
    void onTimerToF();

    // タイマー割り込みフラグの状態を確認する関数
    bool getTmConFlag();
    bool getTmBleFlag();
    bool getTmToFFlag();

    // タイマー割り込みフラグをおろす関数
    void setTmConFlag( bool flag );
    void setTmBleFlag( bool flag );
    void setTmToFFlag( bool flag );

#endif