/*
    BLE通信に関するソースファイル
     地上局との通信（指令とデータロガー）
     ・関連ライブラリのインクルード
     ・関連変数の宣言
     ・関連関数の宣言
*/
#include "../inc/BLE.h"

// BLEの（カスタム）サービスを宣言（固有のUUIDを使う）
static BLEService myS( SERVICE_UUID );
// BLEの（カスタム）キャラクタリスティックを宣言（固有のUUIDを使う）
static BLECharacteristic myC1(CHARACTERISTIC_UUID1, BLERead | BLENotify, 140); // 読み取り属性
static BLECharacteristic myC2(CHARACTERISTIC_UUID2, BLEWrite, 20); // 書き込み属性

// セントラル（接続先）のインスタンスを生成
static BLEDevice central;

static char writtenMsgHead ; // メッセージの先頭文字を格納する変数
static size_t writtenLen ; // メッセージの長さを格納する変数

// BLEの初期設定関数
bool setupBLE() {
  bool state = true; // BLE設定関数のステータス
  if (!BLE.begin()) { //BLEの初期化を実行
    // 失敗ならここへ
    state = false;
  }else{
    //アドバタイズドデータのローカルネーム(local Name)を設定
    BLE.setLocalName(LOCAL_NAME);   

    BLE.setAdvertisedService(myS);  // サービスを立ち上げ
    myS.addCharacteristic(myC1);    // サービスにキャラクタリスティック1を紐づけ
    myS.addCharacteristic(myC2);    // サービスにキャラクタリスティック2を紐づけ
    BLE.addService(myS);  // サービスを追加

    // アドバタイズの開始を指示
    BLE.advertise();

  }
  // 結果を返す（0なら成功）
  return state;
}

// BLEスタックの更新関数
void pollBLE(){
  BLE.poll(); // poll()関数はBLEスタックを更新する
}

// BLEで接続しているセントラル機器を取得
void listenBLE(){
  central = BLE.central();  // セントラル機器を取得する関数
}

// セントラルがペリフェラルに接続されているかどうかの判定関数
//  戻り値：接続されていれば1を，そうでなければ0を返す
bool isConnectedToPeripheral(){
  bool temp = false; // 戻り値の初期化
  if(central){
    temp = true; // 戻り値に真をセット
  }
  return temp;
}

// セントラルへの接続が継続しているかの確認関数
//  戻り値：接続が継続されていれば1を，そうでなければ0を返す
bool centralStillConnected(){
  bool temp = false; // 戻り値の初期化
  if(central.connected()){
    temp = true; // 戻り値に真をセット
  }
  return temp;
}

// キャラクタリスティックにメッセージ（unsigned long）を書き込む関数
void sendULong( unsigned long cnt ){
  // 文字列バッファを定義（バッファサイズは適切に設定）
  char timeString[20];
  // cntの値を文字列に変換
  sprintf(timeString, "%lu", cnt);
  // キャラクタリスティックに書き込み
  sendMessageBLE( timeString );
}

// キャラクタリスティックにメッセージを書き込む関数
//  引数：char配列のmsgのポインタ
void sendMessageBLE( char* msg ){
  myC1.writeValue( msg );
}

// BLEメッセージがあれば受信文字列を更新する関数
//  戻り値：受信メッセージがあれば真を，そうでなければ偽を返す
bool checkWrittenMessage(){
  bool y = false; // 戻り値の初期化

  // キャラクタリスティクスに書き込みがあるかチェック
  if ( myC2.written() ){
    // キャラクタリスティックの値のポインタを取得
    const uint8_t* value = myC2.value();
    // その値の長さを取得
    writtenLen = myC2.valueLength();

    // 値をコピーする用の文字配列を宣言
    char strValue[ writtenLen ];
    // キャラクタリスティックの値をコピーして取得
    memcpy(strValue, value, writtenLen);
    // 取得したメッセージの初めの1文字を取得
    writtenMsgHead = strValue[0] ;

    // 戻り値をtrueに設定
    y = true;
  }
  return y;
}

// 受信メッセージの最初の1文字を返すゲッタ関数
char getWrittenMessageHead(){
  return writtenMsgHead;
}