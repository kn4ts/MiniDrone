#ifndef miniDrone_h
    #define miniDrone_h

    #include "Serial.h" // シリアル通信用のヘッダファイル
    #include "BLE.h"  // BLE通信用のヘッダファイル
    #include "ControlTimer.h" // タイマー用のヘッダファイル
    #include "IntegratedIMU.h"  // 内臓IMU用のヘッダファイル
    #include "SensorI2C.h"  // I2C接続用のヘッダファイル
    #include "Actuator.h" // アクチュエータ（PWM指令）のヘッダファイル

    #include "Control.h" // 制御系のヘッダファイル

    /* ハードウェアピンの設定 */
    #define STATE_DO D2 // 制御周期確認用のDOポート

    /* 参照値指令用カウンタ */
    #define cnt_MAX 30 // [step] (1 step = about 10 ms)
    #define ref_ANGLE 20 // [deg] 角度の目標値の絶対値

    /* 地上局から送られてくるコマンドの定義 */
    enum Command {
        /* 共通コマンド */
        CMD_COMMAND_NONE = 'n', // 何もしないコマンド
        CMD_STOP      = '0',  // 停止コマンド

        CMD_ARM      = 'a',   // アームコマンド
        CMD_CALIBRATE = 'c',  // センサキャリブレーションコマンド

        /* 動作変更コマンド */
        CMD_CONTROL = 's',    // 制御開始コマンド
        CMD_GIMBAL_ROLL_PITCH = 'g',  // 制御開始（ジンバル）コマンド

        CMD_IDLE      = 'i',  // アイドリングコマンド

        /* 動作テストコマンド */
        CMD_TEST_ALL_MOTORS = '1',    // 全モータテストコマンド

        CMD_TEST_ROLL = 'r',  // ロール角動作テストコマンド
        CMD_TEST_PITCH = 'p', // ピッチ角動作テストコマンド

        /* 目標値コマンド */
        CMD_RIGHT    = '6',   // 右移動コマンド
        CMD_LEFT     = '4',   // 左移動コマンド
        CMD_FORWARD  = '8',   // 前進コマンド
        CMD_BACKWARD = '2'    // 後退コマンド
    };

    /* フライトコントローラの動作モード */
    enum Mode {
        /* 共通モード */
        MODE_NORMAL = 2, // 通常モード
        MODE_STOP = 0, // 停止モード

        /* 一度きり実行動作 */
        MODE_CALIBRATE = 31, // センサキャリブレーションモード
        MODE_ARM = 32, // アームモード

        MODE_FORWARD = 41, // 前進モード
        MODE_BACKWARD = 42, // 後退モード
        MODE_LEFT = 43, // 左移動モード
        MODE_RIGHT = 44, // 右移動モード

        /* 動作 */
        MODE_IDLE = 11, // アイドリングモード
        MODE_CONTROL_STANDBY = 10, // 制御開始の準備モード
        MODE_CONTROL = 12, // 制御実行モード
        MODE_GIMBAL_ROLL_PITCH = 30, // ジンバル制御モード

        MODE_TEST_ALL_MOTORS = 1, // 全モータテストモード
        MODE_TEST_ROLL = 21, // ロール角動作テストモード
        MODE_TEST_PITCH = 22 // ピッチ角動作テストモード
    };

    // 物理ピン関係の変数
    extern bool checkTsDO ; // 制御周期確認用DOポートの状態変数

    // BLE通信用変数
    extern char msgSendBLE[192] ;  // BLEで送信するメッセージの格納変数

    // 制御用変数定義
    extern float* att ;  // 姿勢を格納した配列のポインタ格納用変数
    extern float* anv ;  // 角速度を格納した配列のポインタ格納用変数
    extern float alt ;   // 高度を格納する変数

    extern float* mag ;  // 地磁気センサの計測値を格納した配列のポインタ格納用変数

    extern float* uc_pointer ; // 計算した制御入力（PWM指令値）を格納した配列のポインタ格納用変数
    extern int* up_pointer ; // 実際に印加した制御入力（PWM指令値）を格納した配列のポインタ格納用変数
    extern float uc[4] ; // 計算した制御入力（PWM指令値）を格納する配列
    extern float u0[4] ; // すべての要素が0である制御入力（PWM指令値）を格納する配列

    extern int mode ; // モードを保持するための変数
    extern bool arm ; // アーム状態を保持するための変数

    // static int cnt_alt = 0; // 高度指令値用（不要？）
    extern int cnt_rol; // ロール角度指令値用カウンタ
    extern int cnt_pit; // ピッチ角度指令値用カウンタ

    /*
      関数定義 
    */
    int modeDetectionBLE(); // BLE通信からの指令を読み取り・解釈する関数
    float* setUc( float u1, float u2, float u3, float u4 ); // 制御入力配列（uc）の要素に4つの値をセットする関数
    void setMsgSendBLE( unsigned long t, float* att, float* mag, float alt, float* cf,
                    float alt_f, float rol_f, float pit_f, float yaw_f,
                    float ref_a, float ref_r, float ref_p ); // BLEで送信するメッセージを作成する関数
    void toggleDO(); // 制御周期確認用のDO切り替え関数
    void calibrateSensors(); // センサのバイアス値設定関数

    void updateRollPitchReference( int& cnt_r, int& cnt_p ); // ロール・ピッチ角指令値の更新

    /* アームに関する関数 */
    void armDrone(); // ドローンをアームする関数
    void disarmDrone(); // ドローンをディスアームする関数
    bool isArmed(); // ドローンがアームされているか確認する関数
#endif
