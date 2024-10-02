# Mini Drone

## ハードウェア
+ Arduino nano 33 BLE ... BLEモジュールとIMUが搭載されたマイコン（[https://ssci.to/7667](https://ssci.to/7667)）
+ VL53L0X Time-of-Flight 距離センサモジュール ... ToF測距センサ（[https://ssci.to/2894](https://ssci.to/2894)）

### 電気回路の部品
+ FET ... モータ制御用のトランジスタ．
    - 候補1）2SK4019（[https://eleshop.jp/shop/g/gECF311/](https://eleshop.jp/shop/g/gECF311/)）
    - 候補2）2SK4017（[https://akizukidenshi.com/catalog/g/g107597/](https://akizukidenshi.com/catalog/g/g107597/)）
+ 抵抗 ... マイコン電流調整用．100Ω
+ 抵抗 ... プルダウン用．10kΩ
+ ダイオード ... フライバックダイオード．SB240LES（[https://akizukidenshi.com/catalog/g/g116419/](https://akizukidenshi.com/catalog/g/g116419/)）
+ コンデンサ ... パスコン用．セラミックコンデンサー 47μF16V（[https://akizukidenshi.com/catalog/g/g104917/](https://akizukidenshi.com/catalog/g/g104917/)）
+ レギュレータ ... マイコン電源用．3.3V出力ステップアップレギュレータ？（[https://ssci.to/8681](https://ssci.to/8681)）？

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
    participant DebugDO as DO2<br>(デジタル出力ポート) # デバッグ用DOポート
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

## コードの構成図
機能ごとにソースコードを分けて，メイン（`miniDrone.ino`）から呼び出して使用する構造にする．
これにより，機能の追加実装やテスト，複数人での開発が容易になることが期待できる．
```mermaid
graph TD
    classDef h fill:transparent, stroke:#9370DB
    classDef cpp fill:transparent
    classDef ino fill:transparent

    %% style some-group fill:transparent,stroke:#9370DB
    subgraph BLE [BLE機能のユニット]
        subgraph hBT [BLE.h]
            conBT[定数の定義]
            ~~~ proBT[関数のプロトタイプ宣言]
        end
        subgraph cppBT [BLE.cpp]
            valBT[変数の定義]
            ~~~ funBT[関数の中身の実装]
        end
        hBT --インクルード--> cppBT
        %%funBT --予め定義--> proBT
    end
    subgraph ControlTimer [Timer機能のユニット]
        subgraph hCT [ControlTimer.h]
            conCT[定数の定義]
            ~~~ proCT[関数のプロトタイプ宣言]
        end
        subgraph cppCT [ControlTimer.cpp]
            valCT[変数の定義]
            ~~~ funCT[関数の中身の実装]
        end
        hCT --インクルード--> cppCT
    end
    subgraph UnitX [・・・機能のユニット]
        subgraph hX [xxx.h]
            conX[定数の定義]
            ~~~ proX[関数のプロトタイプ宣言]
        end
        subgraph cppX [xxx.cpp]
            valX[変数の定義]
            ~~~ funX[関数の中身の実装]
        end
        hX --インクルード--> cppX
    end
    subgraph miniDrone[miniDrone.ino]
        inc1["#include "必要な機能1""]
        inc2["#include "必要な機能2""]
        incx["#include "必要な機能xxx""]

        subgraph Global[グローバル変数・関数]
            global[グローバル変数の定義]
            ~~~ globalfun[関数の定義]
        end

        subgraph loop[loop関数]
            callfunX[関数の使用]
        end
        
        setup[setup関数]

        inc1 -.- inc2
        inc2 -.- incx
        incx -.-> setup

        setup --> loop
        %%loop --ループ--> loop

        Global --> loop
        funX -.-関数の呼び出し-.-> callfunX
        %%funX -.-関数の呼び出し-.-> setup

        hBT --インクルード--> inc1
        hCT --インクルード--> inc2
        hX --インクルード--> incx
    end
```

## 電気回路の構成
モータ1つ分の駆動系の回路を以下に示す．
![回路図](./doc/circuit_one_motor.png)


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

### 駆動系の電気回路は下記を参考にした
+ https://www.remma.net/?p=1341

### COMが認識されなくなる問題のトラブルシュート  
+ https://detail.chiebukuro.yahoo.co.jp/qa/question_detail/q11263532971

### Mermaidの参考ページ
+ https://qiita.com/run1000dori/items/90f91687cfe7ece50020

### VSCodeでDrawioを使って作図する拡張機能について
+ https://otepipi.hatenablog.com/entry/2020/05/25/192844