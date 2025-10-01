clc, close all, clear all

% クラスのパスを追加
addpath 'MatlabBLE' 	% BLE通信用クラスのパスを追加
addpath 'DataFile' 	% データロガー用クラスのパスを追加
addpath 'DataHandle' 	% データハンドルクラスのパスを追加
addpath 'Timer'		% タイマークラスのパスを追加
addpath 'App'		% Appクラスのパスを追加

addpath 'HoriRap' % HoriRapクラスのパスを追加

% データロガーの設定
OUTPUT_FOLDER = "./output/"; % データロガーの出力用フォルダを指定
df = DataFile( OUTPUT_FOLDER ) % データロガークラスのインスタンス生成

% BLE通信の設定
%ID = "8DFC031CAF32"; % Bluetooth MAC アドレス
% ID = "5BEE875C506D"; % 接続先のドローンの Bluetooth MAC アドレス
% ID = "6D09F5206CBC";
ID = "7A92696EC856";
mble = MatlabBLE( ID )	% BLE通信のインスタンス生成

f = genCallbackFunction( mble, df ); % BLE受信により起動させるコールバック関数を生成
mble.chara_read.DataAvailableFcn = f; % BLEデータ受信時のコールバック関数を設定

% タイマー機能の設定
EXP_TIME = 50 ;	% 最大実験時間の設定[s]
tm = Timer( 1, EXP_TIME );	% 割り込み周期[s]，実行回数[-]

% キー割り込み用のクラス
app = App();

% HoriRapクラスのインスタンス生成
rap = HoriRap();

% =======================
% 送信コマンドの定義
% 	1. 送信コマンドは，マイコン側の動作を規定するコマンドとする
%   2. 送信コマンドは，マイコン側で定義されているコマンド"Command"と合わせる必要がある
% =======================
COMMAND  = dictionary(); % 辞書型としてCOMMANDを初期化
COMMAND("none")		= 'n'; % 何もしないコマンド
COMMAND("stop")		= '0'; % 停止コマンド

COMMAND("arm")		= 'a'; % アームコマンド
COMMAND("calib")	= 'c'; % キャリブレーションコマンド

% 動作変更コマンド
COMMAND("control")	= 's'; % 制御開始コマンド
COMMAND("gimbal")	= 'g'; % ジンバル制御開始コマンド

COMMAND("idle")		= 'i'; % アイドリング（モーター回転）コマンド

% 動作テストコマンド
COMMAND("test_all_motors") = '1'; % 全モーター動作テストコマンド

COMMAND("test_roll")	= 'r'; % ロール軸方向の動作テストコマンド
COMMAND("test_pitch")	= 'p'; % ピッチ軸方向の動作テストコマンド

% 目標値変更コマンド
COMMAND("forward")	= '8'; % 前進指令
COMMAND("back")		= '2'; % 後退指令
COMMAND("left")		= '4'; % 左移動指令
COMMAND("right")	= '6'; % 右移動指令

COMMAND("roll_plus")	= 'R'; % ロール角目標値を増加
COMMAND("roll_minus")	= 'A'; % ロール角目標値を減少
COMMAND("pitch_plus")	= 'P'; % ピッチ角目標値を増加
COMMAND("pitch_minus")	= 'L'; % ピッチ角目標値を減少

COMMAND("att_neutral")	= 'H'; % 姿勢目標値を中立に戻すコマンド

% 未定義コマンド
%COMMAND("disarm")	= 'd'; % ディスアーム（モーター停止）コマンド
%COMMAND("SAFE")	= 'e'; % セーフモード（安全停止モード）への移行コマンド

cmd = COMMAND("none"); % デフォルトは"none"コマンド

% =======================
% 使用するキーの定義
% 	1. 左辺は名称（任意の文字列），右辺はキーボードのキー名
% =======================
KEY = dictionary(); % 辞書型としてKEYを初期化
KEY("c")		= 'c';		% キャリブレーション指令キー
KEY("up")		= 'uparrow';	% 前進指令キー
KEY("down")		= 'downarrow';	% 後退指令キー
KEY("left")		= 'leftarrow';	% 左移動指令キー
KEY("right")	= 'rightarrow';	% 右移動指令キー

% =======================
% 変数の初期化
% =======================
i = 0 ; % カウンタ

%=======================
%	メインループ
%=======================
while( tm.t.Running == "on" ) % タイマーが有効である間ループ

	% ジョイスティックの入力チェック
	rap.updateJoyState();	% ジョイスティックの状態更新
	if rap.checkPOVsChangedXdirection()	% POV（十字キー）のX方向の変化があれば
		povX = rap.getPOVXdirection();	% POVのX方向の状態を取得
		switch povX
			case 1 % right
				cmd = COMMAND("roll_plus"); % ロール軸の目標値を正に
			case -1 % left
				cmd = COMMAND("roll_minus"); % ロール軸の目標値を負に
			otherwise
				cmd = COMMAND("att_neutral"); % ロール軸の目標値を水平に
		end
		if cmd ~= COMMAND("none")	% 送信コマンドが"none"でなければ
			mble.sendMessage( cmd );	% BLE通信でコマンド送信
			str = "POV X direction changed! New POVs: " + povX + ", command sent: " + cmd ;
			disp( str );	% 画面表示
			cmd = ''; % 送信コマンドをリセット
		end
		pause(0.1);	% 0.1秒待つ
	end

	% キー入力のチェック
	if app.getReadFlag() > 0
		keyPressed = app.getReadChara(); % 押されたキーを取得
		str = "key pressed ... " + keyPressed ;
		disp( str );

		% 押されたキーに応じた指令をセット
		switch keyPressed
			% 基本動作モード変更の指令
			case KEY("c")
				cmd = COMMAND("calib"); % キャリブレーション指令をセット

			% 目標値変更の指令
			case KEY("down")
				cmd = COMMAND("back"); % 後退指令
			case KEY("up")
				cmd = COMMAND("forward"); % 前進指令
			case KEY("left")
				cmd = COMMAND("left"); % 左移動指令
			case KEY("right")
				cmd = COMMAND("right"); % 右移動指令

			% 上記以外のキーが押された場合
			otherwise
				break;	% それ以外ならループ抜ける -> 停止指令
		end

		% 指令を送信
		mble.sendMessage( cmd );	% BLE通信でメッセージ送信
		cmd = '';	% 送信コマンドをリセット

		pause(0.1);	% 一時停止
		app.setReadFlag(0); % キー入力フラグおろす
	end

	% タイマー間隔で実行する部分
	if tm.getFlagVal() > 0 % タイマーフラグをチェック
		tm.setFlagVal(0); % タイマーフラグおろす
		i = i +1; % カウンタをインクリメント

		% 画面表示用の設定
		[ data, time, s ] = mble.getReadData();	% BLEの受信メッセージを取得
		str = i +": "+ s + ", " + char(data) ; 	% 文字列の整形
		disp(str); % 画面表示

		% ループ回数の途中でメッセージ送信（BLE通信）
		switch i
			case 2	% 2秒後に
				cmd = COMMAND("calib");  % キャリブレーション指令をセット
				mble.sendMessage( cmd ); % 指令送信
			case 5 % 5秒後に
				cmd = COMMAND("calib"); % 再度キャリブレーション指令をセット
				mble.sendMessage( cmd ); % 指令送信
			case 6 % 6秒後に
				% !! ↓のArmコマンドを送信するとプロペラが回転する可能性があるので注意 !!
				cmd = COMMAND("arm"); % arm状態コマンド
				mble.sendMessage( cmd ); % 指令送信
			case 7 % 7秒後に
				% !! ↓のアイドリングコマンドを送信するとプロペラが回転するので注意 !!
				cmd = COMMAND("idle"); % アイドリングコマンド
				mble.sendMessage( cmd ); % 指令送信
			case 8 % 8秒後に
				% !! ↓の制御開始コマンドを送信するとプロペラが回転するので注意 !!
				cmd = COMMAND("gimbal"); % ジンバル制御開始コマンド
				mble.sendMessage( cmd ); % 指令送信
			case 12
				cmd = COMMAND("roll_plus"); % ロール角目標値を増加
				mble.sendMessage( cmd ); % 指令送信
			case 17
				cmd = COMMAND("roll_minus"); % ロール角目標値を減少
				mble.sendMessage( cmd ); % 指令送信
			case 22
				cmd = COMMAND("att_neutral"); % 姿勢目標値を中立に戻す
				mble.sendMessage( cmd ); % 指令送信
			case 27
				cmd = COMMAND("pitch_plus"); % ピッチ角目標値を増加
				mble.sendMessage( cmd ); % 指令送信
			case 32
				cmd = COMMAND("pitch_minus"); % ピッチ角目標値を減少
				mble.sendMessage( cmd ); % 指令送信
			case 37
				cmd = COMMAND("att_neutral"); % 姿勢目標値を中立に戻す
				mble.sendMessage( cmd ); % 指令送信
			case 42
				cmd = COMMAND("stop"); % 停止指令をセット
				mble.sendMessage( cmd ); % 指令送信
			case 43
				break; % ループ抜ける -> 停止指令
		end
	end

	% アプリ終了判定
	if app.getQuitFlag() > 0  break; end

	pause(0.0001);	% 一時停止
end

% =======================
%  ドローン側の動作停止処理
% =======================
cmd = COMMAND("stop"); % 停止指令をセット
mble.sendMessage( cmd ); % 指令送信
pause(0.5);	% 一時停止

%=======================
%	後処理
%=======================
close gcf; % 図の終了

unsubscribe(mble.chara_read) % データ受信の購読を解除
clear mble	 % BLE通信のインスタンスを削除

%=======================
%	結果表示
%=======================
shapedata()	% データの整形関数の呼び出し
showplot()	% データのプロット関数の呼び出し


%=======================
%	関数定義
%=======================
% BLE受信コールバック関数を生成する高階関数
function f = genCallbackFunction( mble, df )
	
	% コールバック関数の定義
	function callback( src, evt )
		if mble.isReading.getVal()
			disp("BLE is reading");
		else
			mble.isReading.setVal(true); % 読み込みフラグを立てる
			try
				mble.data.setVal( read( src ) );
				%msgBLE = read( src );
			catch ME
				disp("BLE recieve error");
			end
			% 読み込んだデータを変数da（データ）に記録
			% PC時刻を変数ti（タイム）に記録
			mble.time.setVal( MatlabBLE.getDateTimeString() );
			% PC時刻での経過時間（ミリ秒）を変数ti_miに記録
			mble.time_e.setVal( mble.getElapsedTimeString() );
			% データの通し番号をインクリメント
			mble.snum.setVal( mble.snum.getVal() +1 );

			% 保存用文字列の生成
			str = [ ...
					num2str(mble.snum.getVal()), ',', ...	% 受信データの通し番号
					char(mble.time.getVal()), ',', ...	 	% 受信時刻(PC時刻)
					char(mble.time_e.getVal()), ',', ...	% 経過時間(PC時刻)
					char(mble.data.getVal()) ...			% 受信データ
				];

			df.outputDataStr( str ); % データをファイルに出力
			mble.isReading.setVal(false);	% 読み込みフラグをおろす
		end
	end

	f = @callback ; % 定義したコールバック関数を返す
end
