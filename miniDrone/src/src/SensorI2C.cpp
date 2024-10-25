/*
    I2C接続のセンサに関するソースファイル
     高度を測距センサ（I2C接続）で計測する
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/SensorI2C.h"

// ToFセンサ（測距センサ）のインスタンス生成
static VL53L0X senToF;
static uint16_t dist = 0; // 距離計測値の格納用変数
static uint16_t dist_bias = 0; // 距離計測値のバイアス値格納用変数

// BLEの初期設定関数
bool initSensorI2C(){
  bool state = true; // ToFセンサ設定関数のステータス

  Wire.begin();
  senToF.setTimeout(500);

  //if (senToF.init()) { // ToFセンサの初期化を実行
  if (!senToF.init()) { //ToFセンサの初期化を実行
    // 失敗ならここへ
    state = false;
  }else{
    senToF.setMeasurementTimingBudget(29000); // デフォルトは33ms?
    //senToF.setMeasurementTimingBudget(20000); // デフォルトは33ms?
    senToF.startContinuous(); // 連続測定モードを開始
  }
  // 結果を返す（trueなら成功）
  return state;
}

// 高度センサ値の更新関数
void updateAltitudeVal(){
  uint16_t temp = senToF.readRangeContinuousMillimeters(); // 注意：ブロックする

  // タイムアウトや測定不能（8190より大きい値はエラー）でないことを確認
  if ( !senToF.timeoutOccurred() && temp < 8191) {
    dist = temp;
  }
}

void setAltBias(){
  dist_bias = getAltitudeVal();
}
// 高度計測値のゲッタ関数
uint16_t getAltitudeVal(){
  return dist;
}
// 高度計測値のゲッタ関数
float getAltitudeVal_wo_b(){
  return (float)dist - (float)dist_bias;
}