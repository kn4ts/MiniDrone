clc, close all, clear all

addpath 'MatlabBLE' 	% BLE通信用クラスのパスを追加
addpath 'DataFile' 	% データロガー用クラスのパスを追加
addpath 'DataHandle' 	% データハンドルクラスのパスを追加
addpath 'Timer'		% タイマークラスのパスを追加
addpath 'App'		% Appクラスのパスを追加

% データロガーの設定
OUTPUT_FOLDER = "./output/"; % データロガーの出力用フォルダを指定
df = DataFile( OUTPUT_FOLDER ) % データロガークラスのインスタンス生成

% BLE通信の設定
%ID = "8DFC031CAF32"; % Bluetooth MAC アドレス
ID = "5BEE875C506D"; % Bluetooth MAC アドレス
mble = MatlabBLE( ID )	% BLE通信のインスタンス生成

f = genCallbackFunction( mble, df ); % BLE受信により起動させるコールバック関数を生成
mble.chara_read.DataAvailableFcn = f;

% タイマー機能の設定
EXP_TIME = 30 ;	% 最大実験時間の設定[s]
tm = Timer( 1, EXP_TIME );	% 割り込み周期[s]，実行回数[-]

% キー割り込み用のクラス
app = App();

%N = 50;	% ループ回数を設定
i = 0 ; % カウンタ
%=======================
%	メインループ
%=======================
while( tm.t.Running == "on" ) % タイマーが有効である間ループ
%for i=1:N
	if app.getReadFlag() > 0
		app.setReadFlag(0); % フラグおろす
		keyPressed = app.getReadChara() % 押されたキーを取得

		if keyPressed ~= "" break; end
	end

	% タイマー間隔で実行する関数
	if tm.getFlagVal() > 0
		tm.setFlagVal(0); % フラグおろす
		i = i +1;

		% 画面表示用の設定
		[ data, time, s ] = mble.getReadData();	% BLEの受信メッセージを取得

		str = i +": "+ s + ", " + char(data) ; 	% 文字列の整形
		disp(str);

		% ループ回数の途中でメッセージ送信（テスト）
		switch i
			case 3	% キャリブレーション
				mble.sendMessage('c');	% BLE通信でメッセージ送信
			case 5 % アーム
				mble.sendMessage('a');
			case 7 % 制御開始
				mble.sendMessage('s');
		end
	end

	% アプリ終了判定
	if app.getQuitFlag() > 0  break; end

	pause(0.0001);	% 一時停止
end

mble.sendMessage('d');	% 停止指令を送信
pause(0.5);	% 一時停止

close gcf; % 図の終了

unsubscribe(mble.chara_read)
clear mble

%=======================
%	関数定義
%=======================
% BLE受信コールバック関数を生成する高階関数
function f = genCallbackFunction( mble, df )
	
	% コールバック関数の定義
	%function callback( src, evt, da, ti, se )
	function callback( src, evt )
		% 読み込んだデータを変数da（データ）に記録
		mble.data.setVal( read( src ) );
		% PC時刻を変数ti（タイム）に記録
		mble.time.setVal( MatlabBLE.getDateTimeString() );
		% データの通し番号をインクリメント
		mble.snum.setVal( mble.snum.getVal() +1 );

		str = [ num2str(mble.snum.getVal()), ',', char(mble.time.getVal()), ',', char(mble.data.getVal()) ];

		df.outputDataStr( str );
	end
	f = @callback ;
end
