/* ==================================================================
  メインのソースコード
================================================================== */
#include "src/inc/miniDrone.h" // 全体のヘッダファイル


/* ==================================================================
  セットアップ関数
================================================================== */
void setup() {
  /* 出力ピンの設定 */
  pinMode( LED_BUILTIN, OUTPUT ); // 内臓LEDの設定（デバッグ用）
  pinMode( STATE_DO, OUTPUT );  // 制御周期確認用DOポートの設定（デバッグ用）
  setupPWMpin(); // PWM出力ピンの初期化

  /* シリアル通信の初期設定 */
  openSerial();

  /* BLEの初期設定 */
  if( !setupBLE() ){
    // 失敗したらエラー表示で止まる
    while(1){
      Serial.println("BLE setup error!");
      delay(2000);
    };
  }

  /* タイマーの初期設定 */
  setupTimer();

  /* 内臓IMUの初期化 */
  if( !initIMU() ){
    // 失敗したらエラー表示で止まる
    while(1){
      Serial.println("IMU setup error!");
      digitalWrite( LED_BUILTIN, HIGH );
      delay(500);
      digitalWrite( LED_BUILTIN, LOW );
      delay(500);
    };
  }

  /* I2Cセンサの初期化 */
  if( !initSensorI2C() ){
    // 失敗したらエラー表示
    Serial.println("I2C sensor setup error!");
    delay(2000);
  }

}

/* ==================================================================
  メインループ関数
================================================================== */
void loop() {
  /* BLE接続を試行する */
  listenBLE(); // セントラル機器との接続確認
  pollBLE(); // BLEスタックの更新

  /* BLE接続されていたら実行される部分 */
  if ( isConnectedToPeripheral() ){ // セントラルがペリフェラルに接続されているかの確認
    /* BLEセントラルに接続されている間ループ */
    while ( centralStillConnected() ){

      // BLE通信の指令の受信・解釈
      mode = modeDetectionBLE();

      // モードに応じた動作（一度きり実行）
      switch (mode){
        // 制御開始準備
        case MODE_CONTROL_STANDBY:  // mode が CONTROL_STANDBY なら
          setAltitudeReference(300); mode = MODE_CONTROL; break; // 高度目標値をセット・モードをCONTROLへ移行
        
        // センサキャリブレーション
        case MODE_CALIBRATE_SENSORS: // センサキャリブレーション
          calibrateSensors();
          //calibrateGyroBias(); // ジャイロセンサのバイアスを再取得
          mode = MODE_NORMAL; break;

        // 姿勢角キャリブレーション
        case MODE_CALIBRATE_ATTITUDE: // 姿勢角キャリブレーション
          calibrateAttitude();
          mode = MODE_NORMAL; break;

        // Arm
        case MODE_ARM:
          armDrone();
          //arm = true;
          initializeController(); // 制御器をリセット
          mode = MODE_NORMAL; break;

        /* 目標値セット */
        //case MODE_FORWARD:
        //  cnt_pit = -cnt_MAX; mode = MODE_NORMAL; break; // カウンタを-最大値へ
        //case MODE_BACKWARD:
        //  cnt_pit =  cnt_MAX; mode = MODE_NORMAL; break; // カウンタを最大値へ
        //case MODE_LEFT:
        //  cnt_rol = -cnt_MAX; mode = MODE_NORMAL; break; // カウンタを最大値へ
        //case MODE_RIGHT:
        //  cnt_rol =  cnt_MAX; mode = MODE_NORMAL; break; // カウンタを最大値へ

        /* 停止モード */
        case MODE_STOP: 
          disarmDrone();
          break;

        /* 上記以外の場合はスルー */
        default:
          //initializeController(); // 制御器をリセット
          //uc_pointer = setUc( 0, 0, 0, 0 ); // 制御器出力をすべて0に
          break;
      }


      /* -------------------------------
         制御周期タイマー処理のはじまり
      ------------------------------- */
      if ( getTmConFlag() ){
        setTmConFlag(false); // タイマー割込みフラグをおろす

        /*
          指令値カウンタの確認・処理
        */ 
        updateRollPitchReference( cnt_rol, cnt_pit ); // ロール・ピッチ角指令値の更新

        // IMUセンサ値を用いた姿勢角の更新
        //updateIMUAttitudeVal();
        updateIMUAttitudeVal_ver2();

        // IMUで計算した値を取得
        att = getIMUAttitude_wo_b(); // 姿勢を取得
        anv = getIMUAngularVelocity_wo_b(); // 角速度を取得
        // 地磁気計測値を取得
        // mag = getIMUMag(); 

        /* ToFセンサから届いている最新の高度を取得 */
        if ( getToFFlag() ){
          setToFFlag(false); // ToFセンサフラグをおろす
          // 注意：測定値が準備できていないとブロックする
          //updateAltitudeVal();
          updateAltitudeVal();
          alt = getFilteredAltitudeVal_wo_b();
        }

        /*
          高度・姿勢の異常検知
        */
        if( abs(att[0]) > 90 ){ mode = MODE_STOP; }; // ロール角が異常ならモードを0に
        if( abs(att[1]) > 90 ){ mode = MODE_STOP; }; // ピッチ角が異常ならモードを0に

        /*
          モードに応じた動作（継続実行）
          モードの一つとして制御則を実装する
        */
        switch (mode){
          /* 停止モード */
          case MODE_STOP: // mode が 0 ならロータ停止
            disarmDrone();  // ドローンをディスアーム
            uc_pointer = setUc( 0, 0, 0, 0 ); break;

          /* 全モータの動作テストモード */
          case MODE_TEST_ALL_MOTORS:
            uc_pointer = setUc( 20, 20, 20, 20 ); break; // 全モータをPWM値20で回す指令
          
          /* 制御モード */
          case MODE_CONTROL: // mode が 10 なら制御実行
            // 制御モード時の異常検知
            if( alt > 2000 ){ mode = MODE_STOP; };       // 高度計測値が異常ならモードを0に

            // 制御則の計算
            uc_pointer = controller_demo( att, alt ); // 制御則関数
            uc[0] = uc_pointer[0]; // 制御器出力をucにセット
            uc[1] = uc_pointer[1];
            uc[2] = uc_pointer[2];
            uc[3] = uc_pointer[3];
            break;
          
          /* ジンバル制御モード */
          case MODE_GIMBAL_ROLL_PITCH: // 2DoFジンバル（ロールピッチ軸）の姿勢制御モード
            //uc_pointer = gimbalControl_demo( att, alt ); // ジンバル使用時の制御則
            uc_pointer = gimbalControl_demo( att, anv, alt ); // ジンバル使用時の制御則
            uc[0] = uc_pointer[0]; // 制御器出力をucにセット
            uc[1] = uc_pointer[1];
            uc[2] = uc_pointer[2];
            uc[3] = uc_pointer[3];
            break;
          
          /* アイドリングモード */
          case MODE_IDLE:
            uc_pointer = idle_thrust( ); // アイドル推力をセット
            uc[0] = uc_pointer[0]; // アイドリング入力をucにセット
            uc[1] = uc_pointer[1];
            uc[2] = uc_pointer[2];
            uc[3] = uc_pointer[3];
            break;

          /* ロール角動作の確認モード */
          case MODE_TEST_ROLL:
            uc_pointer = test_roll( att );
            uc[0] = uc_pointer[0]; // 入力をucにセット
            uc[1] = uc_pointer[1];
            uc[2] = uc_pointer[2];
            uc[3] = uc_pointer[3];
            break;

          /* ピッチ角動作の確認モード */
          case MODE_TEST_PITCH:
            uc_pointer = test_pitch( att );
            uc[0] = uc_pointer[0]; // 入力をucにセット
            uc[1] = uc_pointer[1];
            uc[2] = uc_pointer[2];
            uc[3] = uc_pointer[3];
            break;
          
          /* 上記以外のモード時 */
          default: // mode のデフォルト設定
            uc_pointer = setUc( 0, 0, 0, 0 ); break; // 全入力を0に
        }

        // アクチュエータを駆動する
        if( isArmed() == true ){
          up_pointer = driveActuator( uc_pointer ); // 制御器出力でアクチュエータを駆動
        }else{
          up_pointer = driveActuator( &u0[0] ); // 入力0で駆動
        }

        // デバッグ用DOポートをトグル -> 100Hz出ているか確認用
        toggleDO();
      }
      /* -------------------------------
         制御周期タイマー処理ここまで
      ------------------------------- */

      /* -------------------------------
         BLE通信用タイマー処理のはじまり
      ------------------------------- */
      if ( getTmBleFlag() ){ 
        setTmBleFlag(false); // BLE割り込みフラグをおろす

        // マイコンの中の時刻を取得
        unsigned long currentTime = millis();

        // 制御器内部の情報を取得
        float* control_force = getControlForceReq(); // 要求制御力を取得
        //float alt_fil = getFilteredDistanceVal_wo_b(); // フィルタ処理後の高度を取得
        float alt_wo_b = getAltitudeVal_wo_b(); // バイアス処理後の高度を取得
        float rol_fil = getRollFiltered(); // フィルタ処理後のロール角を取得
        float pit_fil = getPitchFiltered(); // フィルタ処理後のピッチ角を取得
        float yaw_fil = getYawFiltered(); // フィルタ処理後のヨー角を取得
        float ref_alt = getAltitudeReference(); // 高度指令値を取得
        float ref_rol = getRollReference(); // ロール角指令値を取得
        float ref_pit = getPitchReference(); // ピッチ角指令値を取得

        // BLE通信の送信メッセージを作成
        setMsgSendBLE( currentTime, att, anv, alt_wo_b, control_force,
                   alt, rol_fil, pit_fil, yaw_fil,
                   ref_alt, ref_rol, ref_pit ); // 送信メッセージ作成
        sendMessageBLE(msgSendBLE); // メッセージ送信

        // BLE通信の受信メッセージを確認
        char msgRecvBLE = getWrittenMessageHead(); // 1文字のメッセージを取得

        // シリアル通信でメッセージ送信（デバッグ用）
        //Serial.print(msgRecvBLE); Serial.print(", "); Serial.print(getToFFlag()); Serial.print(", "); Serial.println(alt);
      }
      /* -------------------------------
         BLE通信用タイマー処理ここまで
      ------------------------------- */
    }
  }

  mode = MODE_STOP; // モードをリセット
  disarmDrone(); // ドローンをディスアーム
  uc_pointer = setUc( 0, 0, 0, 0 ); // 入力を初期化
  up_pointer = driveActuator( &u0[0] ); // モータ止める

  printNoCentral( getBLEAddress() ); // セントラル機器がないことをシリアルで表示
  delay(4000);
}