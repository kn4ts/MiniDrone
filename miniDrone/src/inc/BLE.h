/*
    BLE通信に関するヘッダファイル
     ・関連ライブラリのインクルード
     ・関数のプロトタイプ宣言
*/
#ifndef BLE_h // 複数回読み込みを防ぐif def文
    #define BLE_h

    #include <ArduinoBLE.h>  //ArduinoBLEライブラリを使うための宣言

    // BLEの検索時に表示される名前の設定
    #define LOCAL_NAME "Mini Drone BLE" // 個体毎に変えた方がわかりやすいかも

    // サービスとキャラクタリスティックのUUID設定（固有のIDを使う）
    #define SERVICE_UUID            "7e57283a-6d54-4c1b-9b19-1f0c438a81bc"
    #define CHARACTERISTIC_UUID1    "1f9b4ea7-80b0-4c02-95b7-5c8e739a08f6"
    #define CHARACTERISTIC_UUID2    "d2e5cbb1-7f7e-4d3d-93f6-792d7e0f70db"

    /*
        関数のプロトタイプ宣言
    */
    // BLE通信を始める関数
    bool setupBLE();
    // BLEのスタック更新関数
    void pollBLE();
    // セントラル機器との接続確認関数
    void listenBLE();

    // メッセージを送信する関数
    void sendMessageBLE(char* msg);

    void sendULong(unsigned long cnt);

    // 接続確認関数
    bool isConnectedToPeripheral();
    bool centralStillConnected();

    // セントラルのアドレスのゲッタ
    String getBLEAddress();

    // セントラル側から書き込まれたメッセージを確認する関数
    bool checkWrittenMessage();
    // 書き込まれたメッセージのゲッタ関数
    char getWrittenMessageHead();
#endif