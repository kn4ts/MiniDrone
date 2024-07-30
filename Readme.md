# Mini Drone

## ハードウェア
+ Arduino nano 33 BLE ... BLEユニットとIMUが搭載されたマイコン（https://ssci.to/7667）
+ VL53L0X Time-of-Flight 距離センサモジュール ... ToF測距センサ（https://ssci.to/2894）
+ MOS FET？ ... モータ制御用のトランジスタ．2SK4019？（https://eleshop.jp/shop/g/gECF311/）．

## 使用するライブラリ
+ `ArduinoBLE.h` ... BLE(Bluetooth Low Energy)を使用するためのライブラリ．
+ `Arduino_LSM9DS1.h` ... 9軸IMUのライブラリ．
+ `VL53L0X.h` ... ToF(Time of Flight)測距センサのライブラリ．

## 環境構築
1. Arduino IDE をインストール
2. Arduino IDE のボードマネージャから上記ライブラリをインストール  
参考：https://qiita.com/konikoni428/items/f4c7154ccf1eafd331c9

上記に加えて，BLE通信の動作確認のためにターミナルが使えると便利です．たとえばアプリの`LightBlue`など．

## 制御系のブロック線図
制御系のブロック線図の概要を以下に示す．
![ブロック線図](./doc/block.drawio.svg)

## シーケンスの概要図
マイコンの制御用コードを100Hzで実行するために，以下のシーケンス構成を考えた．

```mermaid
sequenceDiagram # シーケンス図の定義
    autonumber # 信号に番号をつける

    # 各要素の定義
    participant DebugDO as DO2 # デバッグ用DOポート
    participant Sen as 測距センサ<br>(VL53L0X) # I2C接続
    participant Micon as マイコン
    participant PC as 地上局PC  # PC（地上局）

    Micon ->> Sen : I2C接続，周期設定
    PC ->> Micon: BLE接続

    loop BLE接続の持続 # ループの定義

        alt 制御用タイマーのフラグがオン # if文
            Note over Micon : IMU計測値から姿勢更新・取得
            Note over Micon : 現在の高度を取得
            # rect rgba(255, 0, 255, 0.2) # 色付け
            Note over Micon : 制御則の計算
            # end
            Note over Micon : PWM出力値更新 => FETへ

            Micon -->> DebugDO : DOポートをトグル（デバッグ用）
            Note over DebugDO : オシロスコープで100Hz出ているか確認
        end

        alt BLE通信用タイマーのフラグがオン
            Note over Micon : 現在の各内部変数からメッセージ作成
            # rect rgba(255, 0, 255, 0.2) # 色付け
            Micon ->> PC : BLE通信，メッセージ送信
            opt データロガー継続
                Note over PC : ファイルへ書き出し
            end
            # end
            Micon -->> PC : シリアル通信（デバッグ用）
        end

        alt 測距センサ用タイマーのフラグがオン
            Micon ->>+ Sen : I2C通信，測定値を要求
            Note over Sen : 注意：未準備だとブロック
            Sen ->>- Micon : I2C通信，測定値を返す
        end

        opt BLE通信による指令送信
            PC ->> Micon : BLE通信，指令送信
            Note over Micon : 指令解釈・モード変更
        end
    end

```
## 動作確認環境


## トラブルシュート
### デバイスマネージャで COM xx が認識されない
下記を試す．  
+ PCの再起動
+ Arduino IDE のバージョン更新確認
+ Arduino IDE でボード情報の更新
+ マイコンのリセットボタンを押しながら接続

コードが間違っていてマイコン側のUSB Serialが機能しなくなっている可能性がある．その場合は以下を試す．
+ マイコンのリセットボタンをダブルタップ -> 違うCOMポートとして認識 -> 認識したCOMポートで正しいコードを書き込み

## References
### フォルダ構成は下記を参考にした  
+ https://qiita.com/somehiro/items/6a6d954be159b6a5fccd  
+ https://zenn.dev/skou/articles/7a49ca2f9f0fcc

### COMが認識されなくなる問題のトラブルシュート  
+ https://detail.chiebukuro.yahoo.co.jp/qa/question_detail/q11263532971

### Mermaidの参考ページ
+ https://qiita.com/run1000dori/items/90f91687cfe7ece50020

### VSCodeでDrawioを使って作図する拡張機能について
+ https://otepipi.hatenablog.com/entry/2020/05/25/192844