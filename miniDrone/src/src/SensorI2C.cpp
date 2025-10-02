/*
    I2C接続のセンサに関するソースファイル
     高度を測距センサ（I2C接続）で計測する
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/SensorI2C.h"

// ToFセンサ（測距センサ）の変数
static VL53L0X senToF;
static uint16_t dist = 0; // 距離計測値の格納用変数
//static uint16_t dist_bias = 0; // 距離計測値のバイアス値格納用変数

static float alt = 0; // 高度計測値の格納用変数
static float alt_bias = 0; // 高度計測値のバイアス値格納用変数
static float alt_filt = 0; // 高度計測値のローパスフィルタ処理後の値

static float alpha = 0.15; // ローパスフィルタの係数

static bool flagToF = false; // 計測値の準備状態のフラグ

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
    //senToF.setMeasurementTimingBudget(29000); // 計測周期を設定，デフォルトは33ms?
    //senToF.setMeasurementTimingBudget(20000); // デフォルトは33ms?

    /* ↓機能しているかよくわからない部分（なしでもGPIOピンが測定完了時にLOWにできてる？）*/
    // GPIO1を割り込みとして設定する
    // 0x0A4: GPIO_HV_MUX_ACTIVE_HIGH レジスタ
    // 0x01: Active Low 設定（LOWレベルで割り込み発生）
    writeRegister(0x0A4, 0x01);
    // 0x0F0: SYSTEM__INTERRUPT_CONFIG_GPIO レジスタ
    // 0x04: 測定完了時に割り込みを発生させる設定
    writeRegister(0x0F0, 0x04);
    // 割り込みをクリア
    writeRegister(0x0F1, 0x01); // SYSTEM__INTERRUPT_CLEAR レジスタ
    /* ↑機能しているかよくわからない部分（なしでもGPIOピンが測定完了時にLOWにできてる？）*/

    senToF.startContinuous(); // 連続測定モードを開始

    // 割り込みピンの設定
    pinMode( VL53L0X_IR, INPUT_PULLUP );
    // FALLINGエッジで割り込む関数を設定
    attachInterrupt(digitalPinToInterrupt(VL53L0X_IR), onSensorInterrupt, FALLING);
  }
  // 結果を返す（trueなら成功）
  return state;
}
// 割り込み関数
void onSensorInterrupt() {
  setToFFlag(true); // 割り込み発生フラグを立てる
}
// レジスタ操作用の関数
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x29); // VL53L0Xのデフォルトアドレス
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// 割り込みが発生したときに呼び出される関数
bool getToFFlag(){
  return flagToF;
}
void setToFFlag( bool val ){
  flagToF = val;
}

// 高度センサ値の更新関数
void updateAltitudeVal(){
  uint16_t temp = senToF.readRangeContinuousMillimeters(); // 注意：ブロックする

  // タイムアウトや測定不能（8190より大きい値はエラー）でないことを確認
  if ( !senToF.timeoutOccurred() && temp < 8191) {
    dist = temp;  // 距離計測値（uint）を更新
    alt  = (float)dist; // 高度計測値（float）を更新

    alt_filt = lowpassFilter_demo( alt_filt, alt ); // ローパスフィルタをかける
  }
}

// 距離のローパスフィルタの実装例
float lowpassFilter_demo( float alt_filt, float alt ){
    // 高度計測値に1次のローパスフィルタをかける
    float alt_filt_new = ( 1 - alpha ) * alt_filt + alpha * alt ;
    return alt_filt_new;
}
// 距離計測値のバイアスのセッタ関数
void setAltBias(){
  //alt_bias = getAltitudeVal();
  alt_bias = getFilteredAltitudeVal();
}
// 距離計測値のゲッタ関数
float getAltitudeVal(){ return alt; }
// 距離計測値のゲッタ関数
float getAltitudeVal_wo_b(){
  return alt - alt_bias;
}
// フィルタ後の距離計測値のゲッタ関数
float getFilteredAltitudeVal(){ return alt_filt; }
// フィルタ後のバイアスなし高度計測値のゲッタ関数
float getFilteredAltitudeVal_wo_b(){
  return alt_filt - alt_bias;
}