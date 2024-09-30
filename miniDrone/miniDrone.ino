/*
  メインのソースコード
*/
#include "src/inc/Serial.h" // シリアル通信用のヘッダファイル
#include "src/inc/BLE.h"  // BLE通信用のヘッダファイル
#include "src/inc/ControlTimer.h" // タイマー用のヘッダファイル
#include "src/inc/IntegratedIMU.h"  // 内臓IMU用のヘッダファイル
#include "src/inc/SensorI2C.h"  // I2C接続用のヘッダファイル

// 物理ピン関係の変数
#define STATE_DO D2 // 制御周期確認用のDOポート
static bool checkTsDO = false ; // 制御周期確認用DOポートの状態変数

// BLE通信用変数
static String rstr ;  // BLE受信文字列の宣言
static char msgBLE[50] ;  // BLEで送信するメッセージの格納変数

// 制御用変数定義
static float* vec ;  // 姿勢を格納した配列のポインタ格納用変数
static uint16_t alti ; // 高度を格納する変数

static int mode = 0; // モードを保持するための変数

/* 関数定義 */
// BLEで送信するメッセージを作成する関数
void genMsgBLE( unsigned long t, float* v, uint16_t alti ){
//void genMsgBLE( unsigned long t, float x, float y, float z, uint16_t alti ){
  //sprintf(msgBLE, "%d,%.3f,%.3f,%.3f,%d", t, v[0], v[1], v[2], alti);
  sprintf(msgBLE, "%d,%.3f,%.3f,%.3f,%d,%d", t, v[0], v[1], v[2], alti, mode);
}

// 制御周期確認用のDO切り替え関数
void toggleDO(){
  checkTsDO = !checkTsDO ;  // フラグを切り替え
  if( checkTsDO ){
    digitalWrite( STATE_DO, HIGH ); // DOピンをHIGHに
  }else{
    digitalWrite( STATE_DO, LOW ); // DOピンをLOWに
  }
}

/* セットアップ関数 */
void setup() {
  // ピンモードの初期設定
  pinMode( LED_BUILTIN, OUTPUT ); // 内臓LEDの設定（デバッグ用）
  pinMode( STATE_DO, OUTPUT );  // 制御周期確認用DOポートの設定（デバッグ用）

  pinMode( D5, OUTPUT );  // D5ピンをOUTPUT（出力）に設定

  // シリアル通信の初期設定
  openSerial();

  // BLEの初期設定
  if( !setupBLE() ){
    // 失敗したらエラー表示で止まる
    while(1){
      Serial.println("BLE setup error!");
      delay(2000);
    };
  }

  // タイマーの初期設定
  setupTimer();

  // 内臓IMUの初期化
  if( !initIMU() ){
    // 失敗したらエラー表示で止まる
    while(1){
      Serial.println("IMU setup error!");
      delay(2000);
    };
  }

  // I2C接続のセンサの初期設定
  /* if( !initSensorI2C() ){
    // 失敗したらエラー表示で止まる
    while(1){
      Serial.println("I2C sensor setup error!");
      delay(2000);
    };
  } */
 digitalWrite(LED_BUILTIN, HIGH);
}

/* メインループ */
void loop() {
  listenBLE(); // セントラル機器との接続確認
  pollBLE(); // BLEスタックの更新

  if ( isConnectedToPeripheral() ){ // セントラルがペリフェラルに接続されているかの確認
    while ( centralStillConnected() ){  // セントラルが接続されている間ループ

      // BLE通信の指令の受信・解釈
      pollBLE(); // BLEスタックの更新
      bool temp = checkWrittenMessage(); // 書き込まれたメッセージを確認
      if( true == temp ){ // 戻り値のチェック，trueなら指令として読み込む
        char cmd = getWrittenMessageHead() ; // 読み込んだ文字を取得
        switch (cmd){
          case '1': // 受信文字が（char型の）'1'なら
            mode = 1; // モードを1に変更
            break;
          case '0': // 受信文字が（char型の）'0'なら
            mode = 0; // モードを0に変更
            break;
          case '2': // 受信文字が（char型の）'0'なら
            mode = 2; // モードを2に変更
            break;
          default:
            break;
        }
      }

      // 制御用タイマー割り込みフラグがオンならif文の中へ
      if ( getTmConFlag() ){
        // フラグをおろす
        setTmConFlag(false);

        // IMUセンサ値を用いた姿勢角の更新
        updateIMUAttitudeVal();
        // IMUで計算した姿勢を取得
        vec = getIMUAttitude();
        // 測距センサから届いている最新の高度を取得
        alti = getAltitudeVal();

        /*
          ここに制御則を実装する
        */
        
        /*
          ここにアクチュエータ駆動のコードを実装する
        */
        if( mode > 1 ){
          analogWrite( D5, 100 ); // D5ピンを100
        }else if( mode > 0 ){
          analogWrite( D5, 255 ); // D5ピンをHIGH(255)に設定
        }else{
          analogWrite( D5, 0 ); // D5ピンをLOW(0)に設定
        }

        // デバッグ用DOポートをトグル -> 100Hz出ているか確認用
        toggleDO();
      }

      // BLE通信用タイマー割り込みフラグがオンならif文の中へ
      if ( getTmBleFlag() ){ 
        setTmBleFlag(false); // タイマー割り込みフラグをおろす

        // マイコンの中の時刻を取得
        unsigned long currentTime = millis();

        // BLE通信
        genMsgBLE( currentTime, vec, alti ); // 送信メッセージ作成
        sendMessageBLE(msgBLE); // メッセージ送信

        // BLE通信の受信メッセージを確認
        char rcvMsg = getWrittenMessageHead(); // 1文字のメッセージを取得

        // シリアル通信でメッセージ送信（デバッグ用）
        Serial.print(rcvMsg);
        Serial.print(", ");
        Serial.println(alti);
      }

      // ToFセンサ用タイマー割り込みフラグがオンならif文の中へ
      if ( getTmToFFlag() ){
        // フラグをおろす
        setTmToFFlag(false);
        // 測距センサ値を用いた高度の更新
        // updateAltitudeVal(); // 注意：測定値が準備できていないとブロックする
      }
    }
  }
  printNoCentral(); // セントラル機器がないことをシリアルで表示
  delay(4000);
}