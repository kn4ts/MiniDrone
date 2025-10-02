#include "../inc/miniDrone.h" // 全体のヘッダファイル

// 物理ピン関係の変数
bool checkTsDO = false ; // 制御周期確認用DOポートの状態変数

// BLE通信用変数
char msgSendBLE[192] ;  // BLEで送信するメッセージの格納変数

// 制御用変数定義
float* att ;  // 姿勢を格納した配列のポインタ格納用変数
float* anv ;  // 角速度を格納した配列のポインタ格納用変数
float alt ;   // 高度を格納する変数

float* mag ;  // 地磁気センサの計測値を格納した配列のポインタ格納用変数

float* uc_pointer ; // 計算した制御入力（PWM指令値）を格納した配列のポインタ格納用変数
int* up_pointer ; // 実際に印加した制御入力（PWM指令値）を格納した配列のポインタ格納用変数
float uc[4] = {0,0,0,0} ; // 計算した制御入力（PWM指令値）を格納する配列
float u0[4] = {0,0,0,0} ; // すべての要素が0である制御入力（PWM指令値）を格納する配列

//int mode = 0; // モードを保持するための変数
int mode = MODE_NORMAL ; // モードを保持するための変数
bool arm = false; // アーム状態を保持するための変数

// static int cnt_alt = 0; // 高度指令値用（不要？）
int cnt_rol = 0; // ロール角度指令値用カウンタ
int cnt_pit = 0; // ピッチ角度指令値用カウンタ

/*
  関数定義 
*/

// BLE通信からの指令を読み取り・解釈する関数
int modeDetectionBLE(){
  pollBLE(); // BLEスタックの更新
  //mode = MODE_NORMAL ; // デフォルトは通常モード
  if( true == checkWrittenMessage() ){ // 書き込まれたメッセージを確認し，trueなら指令として読み込む
    char cmd = getWrittenMessageHead() ; // 読み込んだ先頭文字を取得
    switch (cmd){
      case CMD_STOP: // 受信文字が（char型の）'0'なら
        mode = MODE_STOP; break;
      case CMD_TEST_ALL_MOTORS: // 受信文字が（char型の）'1'なら
        mode = MODE_TEST_ALL_MOTORS; break;
      case CMD_CONTROL: // 受信文字が（char型の）'s'なら
        mode = MODE_CONTROL_STANDBY; break;
      case CMD_IDLE: // 受信文字が（char型の）'i'なら
        mode = MODE_IDLE; break;
      case CMD_TEST_ROLL: // ロール角動作の確認モード
        mode = MODE_TEST_ROLL; break;
      case CMD_TEST_PITCH: // ピッチ角動作の確認モード
        mode = MODE_TEST_PITCH; break;
      case CMD_CALIBRATE_SENSORS: // 受信文字が（char型の）'c'なら
        mode = MODE_CALIBRATE_SENSORS; break;
      case CMD_CALIBRATE_ATTITUDE: // 受信文字が（char型の）'C'なら
        mode = MODE_CALIBRATE_ATTITUDE; break;
      case CMD_GIMBAL_ROLL_PITCH: // 受信文字が（char型の）'g'なら
        mode = MODE_GIMBAL_ROLL_PITCH; break;
      case CMD_GIMBAL_ROLL: // 受信文字が（char型の）'f'なら
        mode = MODE_GIMBAL_ROLL; break;
      case CMD_ARM: // 受信文字が（char型の）'a'ならarm
        mode = MODE_ARM; break;

      // 前進・後退（ピッチ角）
      case CMD_FORWARD: // 受信文字が（char型の）'8'（前進）なら
        //mode = MODE_FORWARD; break;
        cnt_pit = -cnt_MAX; break; // カウンタを-最大値へ
      case CMD_BACKWARD: // 受信文字が（char型の）'2'（後退）なら
        //mode = MODE_BACKWARD; break;
        cnt_pit =  cnt_MAX; break; // カウンタを最大値へ
      // 左・右（ロール角）
      case CMD_LEFT: // 受信文字が（char型の）'4'（左）なら
        //mode = MODE_LEFT; break;
        cnt_rol = -cnt_MAX; break; // カウンタを最大値へ
      case CMD_RIGHT: // 受信文字が（char型の）'6'（右）なら
        //mode = MODE_RIGHT; break;
        cnt_rol =  cnt_MAX; break; // カウンタを最大値へ

      case CMD_ROLL_PLUS: // 受信文字が（char型の）'R'（ロール角目標値増加）なら
        cnt_rol = cnt_MAX_LONG; cnt_pit = 0; break; // ロール角目標値を増加させる
      case CMD_ROLL_MINUS: // 受信文字が（char型の）'A'（ロール角目標値減少）なら
        cnt_rol = -cnt_MAX_LONG; cnt_pit = 0; break; // ロール角目標値を減少させる
      case CMD_PITCH_PLUS: // 受信文字が（char型の）'P'（ピッチ角目標値増加）なら
        cnt_pit = cnt_MAX_LONG; cnt_rol = 0; break; // ピッチ角目標値を増加させる
      case CMD_PITCH_MINUS: // 受信文字が（char型の）'L'（ピッチ角目標値減少）なら
        cnt_pit = -cnt_MAX_LONG; cnt_rol = 0; break; // ピッチ角目標値を減少させる
      case CMD_ATTITUDE_NEUTRAL: // 受信文字が（char型の）'H'（姿勢目標値中立）なら
        cnt_rol = 0 ; cnt_pit = 0 ; break; // 姿勢目標値を中立に戻す

      /* 上記以外の場合は停止 */
      default:
        mode = MODE_STOP; break; // 予期しない文字なら停止モード
    }
  }
  return mode ;
}

// BLEで送信するメッセージを作成する関数
void setMsgSendBLE( unsigned long t, float* att, float* mag, float alt, float* cf,
                float alt_f, float rol_f, float pit_f, float yaw_f,
                float ref_a, float ref_r, float ref_p ){
  //char msgBLE[192] ;  // BLEで送信するメッセージの格納変数
  sprintf(msgSendBLE,
      "%d,"                 // マイコン内時間[ms]
      "%.2f,%.2f,%.2f,"     // 姿勢角（ロール，ピッチ，ヨーの順）
      "%.2f,%.2f,%.2f,"     // フィルタ後の姿勢角（ロール，ピッチ，ヨーの順）
      "%.2f,%.2f,"          // 高度[mm], 高度フィルタ値[mm]
      "%.1f,%.1f,%.1f,%.1f,"// 制御器出力1~4
      "%.1f,%.1f,%.1f,%.1f,"// 要求制御力(ロール，ピッチ，ヨー，総推力の順)
      "%.1f,%.1f,%.1f,"     // 高度指令値，ロール指令値，ピッチ指令値
      "%d,%d",              // モード，アーム状態
      t,
      att[0],att[1],att[2],
      rol_f,pit_f,yaw_f,
      alt, alt_f,
      uc[0],uc[1],uc[2],uc[3],
      cf[0],cf[1],cf[2],cf[3],
      ref_a,ref_r,ref_p,
      mode, arm);
}

// 制御入力配列（uc）の要素に4つの値をセットする関数
float* setUc( float u1, float u2, float u3, float u4 ){
  uc[0] = u1 ; uc[1] = u2 ; uc[2] = u3 ; uc[3] = u4;
  return &uc[0];
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
  calibrateGyroBias(); // ジャイロセンサのバイアスを設定
  setAltBias(); // 距離センサのバイアスを設定
}

// 姿勢角のキャリブレーション関数
void calibrateAttitude(){
  setAttBias();
}

// 目標ロール・ピッチ角の更新関数
void updateRollPitchReference( int& cnt_r, int& cnt_p ){
    // ロール角について
    if( cnt_r > 0 ){
      setRollReference( ref_ANGLE ); // ロール角目標値を＋方向へ
      --cnt_r ;
    }else if( cnt_r < 0 ){
      setRollReference( -ref_ANGLE ); // ロール角目標値をー方向へ
      ++cnt_r ;
    }else{ setRollReference( 0 ); }; // ロール角目標値を0へ
    // ピッチ角について
    if( cnt_p > 0 ){
      setPitchReference( ref_ANGLE ); // ピッチ角目標値を＋方向へ
      --cnt_p ;
    }else if( cnt_p < 0 ){
      setPitchReference( -ref_ANGLE ); // ピッチ角目標値をー方向へ
      ++cnt_p ;
    }else{ setPitchReference( 0 ); }; // ピッチ角目標値を0へ
}

// アーム化する関数
void armDrone(){
    if( isArmed() == true ) return; // すでにアーム化されていれば何もしない
    arm = true;
    digitalWrite( LED_BUILTIN, HIGH);
}

// 非アーム化する関数
void disarmDrone(){
    if( isArmed() == false ) return; // すでに非アーム化されていれば何もしない
    arm = false;
    digitalWrite( LED_BUILTIN, LOW);
}

// アーム状態を取得する関数
bool isArmed(){
  return arm;
}