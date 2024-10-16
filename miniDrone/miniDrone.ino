/*
  メインのソースコード
*/
#include "src/inc/Serial.h" // シリアル通信用のヘッダファイル
#include "src/inc/BLE.h"  // BLE通信用のヘッダファイル
#include "src/inc/ControlTimer.h" // タイマー用のヘッダファイル
#include "src/inc/IntegratedIMU.h"  // 内臓IMU用のヘッダファイル
#include "src/inc/SensorI2C.h"  // I2C接続用のヘッダファイル
#include "src/inc/Actuator.h" // アクチュエータ（PWM指令）のヘッダファイル

#include "src/inc/Control.h" // 制御系のヘッダファイル

// 物理ピン関係の変数
#define STATE_DO D2 // 制御周期確認用のDOポート
static bool checkTsDO = false ; // 制御周期確認用DOポートの状態変数

// BLE通信用変数
static String rstr ;  // BLE受信文字列の宣言
static char msgBLE[50] ;  // BLEで送信するメッセージの格納変数

// 制御用変数定義
static float* att ;  // 姿勢を格納した配列のポインタ格納用変数
static float* anv ;  // 角速度を格納した配列のポインタ格納用変数
static float alt ; // 高度を格納する変数

static float* mag ;

static int mode = 0; // モードを保持するための変数
static bool arm = false; // アーム状態を保持するための変数

static float u[4] ; // 計算した制御入力（PWM指令値）を格納する配列
static float u0[4] ; // すべての要素が0である制御入力（PWM指令値）を格納する配列
static float* uc ; // 実際に印加した制御入力（PWM指令値）を格納した配列のポインタ格納用変数
static int* up ; // 実際に印加した制御入力（PWM指令値）を格納した配列のポインタ格納用変数

/* 指令用カウンタ */
#define cnt_MAX 30 // [step] (1 step = about 10 ms)
// static int cnt_alt = 0; // 高度指令値用（不要？）
static int cnt_rol = 0; // ロール角度指令値用
static int cnt_pit = 0; // ピッチ角度指令値用

/* 関数定義 */
// BLEで送信するメッセージを作成する関数
void genMsgBLE( unsigned long t, float* att, float* mag, float alt, float* cf ){
//void genMsgBLE( unsigned long t, float x, float y, float z, uint16_t alti ){
  //sprintf(msgBLE, "%d,%.3f,%.3f,%.3f,%d", t, v[0], v[1], v[2], alti);
  sprintf(msgBLE,
      "%d," // マイコン内時間[ms]
      "%.2f,%.2f,%.2f," // 姿勢角（ロール，ピッチ，ヨーの順）
      "%.2f," // 高度[mm]
      "%.1f,%.1f,%.1f,%.1f,"  // 制御器出力1~4
      "%.1f,%.1f,%.1f,%.1f,"  // 要求制御力(ロール，ピッチ，ヨー，総推力の順)
      "%d,%d", // モード，アーム状態
      t,
      att[0],att[1],att[2],
      alt,
      uc[0],uc[1],uc[2],uc[3],
      cf[0],cf[1],cf[2],cf[3],
      mode, arm);
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

// センサのキャリブレーション（センサ値のバイアス処理）関数
void calibrateSensors(){
  setAttBias();
  setAnvBias();
  setAltBias();
}

/* セットアップ関数 */
void setup() {
  // ピンモードの初期設定
  pinMode( LED_BUILTIN, OUTPUT ); // 内臓LEDの設定（デバッグ用）
  pinMode( STATE_DO, OUTPUT );  // 制御周期確認用DOポートの設定（デバッグ用）

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
  if( !initSensorI2C() ){
    // 失敗したらエラー表示で止まる
    while(1){
      Serial.println("I2C sensor setup error!");
      delay(2000);
    };
  }

  // アクチュエータ（モータ）の初期設定
  setupPWMpin();

  // 変数初期化
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
          case '0': // 受信文字が（char型の）'0'なら
            mode = 0; // モードを0に変更
            break;
          case '1': // 受信文字が（char型の）'1'なら
            mode = 1; // モードを1に変更
            break;
          case '2': // 受信文字が（char型の）'2'（後退）なら
            cnt_pit = cnt_MAX; // カウンタを最大値へ
            break;
          case '3': // 受信文字が（char型の）'3'なら
            mode = 3; // モードを3に変更
            break;
          case '4': // 受信文字が（char型の）'4'（左）なら
            cnt_rol = -cnt_MAX; // カウンタを最大値へ
            break;
          case '5': // 受信文字が（char型の）'5'なら
            mode = 5; // モードを5に変更
            break;
          case '6': // 受信文字が（char型の）'6'（右）なら
            cnt_rol = cnt_MAX; // カウンタを最大値へ
            break;
          case '7': // 受信文字が（char型の）'7'なら
            mode = 7; // モードを7に変更
            break;
          case '8': // 受信文字が（char型の）'8'（前進）なら
            cnt_pit = -cnt_MAX; // カウンタを-最大値へ
            break;
          /* case 'u': // 受信文字が（char型の）'u'（上昇）なら
            cnt_alt = cnt_MAX; // カウンタを最大値へ
            break;
          case 'd': // 受信文字が（char型の）'d'（下降）なら
            cnt_alt = cnt_MAX; // カウンタを最大値へ
            break; */
          case 's': // 受信文字が（char型の）'s'なら
            mode = 10; // モードを10に変更
            break;
          case 'c': // 受信文字が（char型の）'c'なら
            calibrateSensors();     // センサのバイアス値設定
            initializeController(); // 制御器をリセット
            break;
          case 'a': // 受信文字が（char型の）'a'なら
            if(arm==false){
              //initializeSensorVal();
              arm = true; // モードを9に変更
            }
            break;
          default:
              arm = false; // モードを9に変更
              initializeController(); // 制御器をリセット
            break;
        }
      }

      // 制御用タイマー割り込みフラグがオンならif文の中へ
      if ( getTmConFlag() ){
        // フラグをおろす
        setTmConFlag(false);

        // IMUセンサ値を用いた姿勢角の更新
        updateIMUAttitudeVal();
        // IMUで計算した値を取得
        att = getIMUAttitude_wo_b(); // 姿勢を取得
        anv = getIMUAngularVelocity_wo_b(); // 角速度を取得

        mag = getIMUMag(); // 地磁気計測値を取得
        // 測距センサから届いている最新の高度を取得
        alt = getAltitudeVal_wo_b();

        /*
          指令値カウンタの確認・処理
        */ 
        // ロール角について
        if( cnt_rol > 0 ){
          setRollReference( 2 ); // ロール角目標値を＋方向へ
          --cnt_rol ;
        }else if( cnt_rol < 0 ){
          setRollReference( -2 ); // ロール角目標値をー方向へ
          ++cnt_rol ;
        }else{ setRollReference( 0 ); }; // ロール角目標値を0へ
        // ピッチ角について
        if( cnt_pit > 0 ){
          setPitchReference(  2 ); // ピッチ角目標値を＋方向へ
          --cnt_pit ;
        }else if( cnt_pit < 0 ){
          setPitchReference( -2 ); // ピッチ角目標値をー方向へ
          ++cnt_pit ;
        }else{ setPitchReference( 0 ); }; // ピッチ角目標値を0へ


        /*
          ここに制御則を実装する
        */
        switch (mode)
        {
        case 0: // mode が 0 なら
          u[0] = 0; // 全モータ停止
          u[1] = 0;
          u[2] = 0;
          u[3] = 0;
          break;
        case 1: // mode が 1 なら
          u[0] = 20; // モータ1をPWM値20で回す指令（制御入力1を20に設定）
          u[1] = 20; // モータ2をPWM値20で回す指令（制御入力2を20に設定）
          u[2] = 20; // モータ3をPWM値20で回す指令（制御入力3を20に設定）
          u[3] = 20; // モータ4をPWM値20で回す指令（制御入力4を20に設定）
          break;
        case 10: // mode が 10 なら
          uc = controller_demo( att, alt );
          
          break;
        default: // mode のデフォルト設定
          u[0] = 0;
          u[1] = 0;
          u[2] = 0;
          u[3] = 0;
          break;
        }

        /*
          ここにアクチュエータ駆動のコードを実装する
        */
        if( arm == true ){
          if( mode == 10 ){
            up = driveActuator( uc ); // 制御器出力でアクチュエータを駆動
          }else{
            up = driveActuator( &u[0] );
          }
        }

        // デバッグ用DOポートをトグル -> 100Hz出ているか確認用
        toggleDO();
      }

      // BLE通信用タイマー割り込みフラグがオンならif文の中へ
      if ( getTmBleFlag() ){ 
        setTmBleFlag(false); // タイマー割り込みフラグをおろす

        // マイコンの中の時刻を取得
        unsigned long currentTime = millis();

        // 制御力を取得
        float* control_force = getControlForceReq();

        // BLE通信
        genMsgBLE( currentTime, att, anv, alt, control_force ); // 送信メッセージ作成
        sendMessageBLE(msgBLE); // メッセージ送信

        // BLE通信の受信メッセージを確認
        char rcvMsg = getWrittenMessageHead(); // 1文字のメッセージを取得

        // シリアル通信でメッセージ送信（デバッグ用）
        Serial.print(rcvMsg);
        Serial.print(", ");
        Serial.println(alt);
      }

      // ToFセンサ用タイマー割り込みフラグがオンならif文の中へ
      if ( getTmToFFlag() ){
        // フラグをおろす
        setTmToFFlag(false);
        // 測距センサ値を用いた高度の更新
        updateAltitudeVal(); // 注意：測定値が準備できていないとブロックする
      }

      if( arm == true ){
        digitalWrite( LED_BUILTIN, HIGH);
      }else{
        digitalWrite( LED_BUILTIN, LOW);
      }
    }
  }
  u[0] = 0; // 入力を初期化
  u[1] = 0;
  u[2] = 0;
  u[3] = 0;
  mode = 0; // モードをリセット
  up = driveActuator( &u[0] ); // モータ止める

  printNoCentral(); // セントラル機器がないことをシリアルで表示
  delay(4000);
}