/*
    時間管理のためのタイマに関するソースファイル
      mbed OS のTickerクラスを使用する
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/ControlTimer.h"

// Tickerオブジェクトを定義
static mbed::Ticker tmCon; // 制御周期のタイマー
static mbed::Ticker tmBle; // BLE通信用のタイマー
static mbed::Ticker tmToF; // 測距センサ用のタイマー

// タイマーフラグ変数の宣言
static bool tmConFlag = false;
static bool tmBleFlag = false;
static bool tmToFFlag = false;

// タイマー周期の定義 [s]
static const float TsCon = 0.01 ;
static const float TsBle = 0.1 ; // 100msが最短？
static const float TsToF = 0.03 ;

// 制御用割り込み関数
void onTimerCon() {
  // tmConFlag = true; // フラグを立てる
  setTmConFlag(true) ; // フラグを立てる
}
// BLE通信用割り込み関数
void onTimerBle() {
  setTmBleFlag(true) ; // フラグを立てる
}
// ToFセンサ用割り込み関数
void onTimerToF() {
  setTmToFFlag(true) ; // フラグを立てる
}

// タイマーのセットアップ関数
void setupTimer(){
    //tm.attach(onTimer, 0.5);
    //tm.attach(onTimer, 0.01);
    //tm.attach(onTimer, 0.05);
    tmCon.attach(onTimerCon, TsCon);
    tmBle.attach(onTimerBle, TsBle);
    tmToF.attach(onTimerToF, TsToF);
}

// フラグのゲッタ関数
bool getTmConFlag(){
  return tmConFlag ;
}
bool getTmBleFlag(){
  return tmBleFlag ;
}
bool getTmToFFlag(){
  return tmToFFlag ;
}

// フラグのセッタ関数
void setTmConFlag( bool flag ){
  tmConFlag = flag ;
}
void setTmBleFlag( bool flag ){
  tmBleFlag = flag ;
}
void setTmToFFlag( bool flag ){
  tmToFFlag = flag ;
}