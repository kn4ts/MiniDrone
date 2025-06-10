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
ID = "5BEE875C506D"; % 接続先のドローンの Bluetooth MAC アドレス
mble = MatlabBLE( ID )	% BLE通信のインスタンス生成

f = genCallbackFunction( mble, df ); % BLE受信により起動させるコールバック関数を生成
mble.chara_read.DataAvailableFcn = f; % BLEデータ受信時のコールバック関数を設定

% タイマー機能の設定
EXP_TIME = 3000 ;	% 最大実験時間の設定[s]
tm = Timer( 1, EXP_TIME );	% 割り込み周期[s]，実行回数[-]

% キー割り込み用のクラス
app = App();

%N = 50;	% ループ回数を設定
i = 0 ; % カウンタ
%=======================
%	メインループ
%=======================
while( tm.t.Running == "on" ) % タイマーが有効である間ループ
	% キー入力のチェック
	if app.getReadFlag() > 0
		keyPressed = app.getReadChara(); % 押されたキーを取得
		str = "key pressed ... " + keyPressed ;
		disp( str );

		% 押されたキーに応じた指令送信
		switch keyPressed
			%case 'downarrow'
			%	cmd = '2'; % 後退指令
			%case 'uparrow'
			%	cmd = '8'; % 前進指令
			%case 'leftarrow'
			%	cmd = '4'; % 左移動指令
			%case 'rightarrow'
			%	cmd = '6'; % 右移動指令
			case 'c'
				cmd = 'c'; % キャリブレーション
			otherwise
				break;	% それ以外ならループ抜ける
		end

		mble.sendMessage( cmd );	% BLE通信でメッセージ送信
		cmd = '';
		pause(0.1);	% 一時停止
		app.setReadFlag(0); % フラグおろす
	end

	% タイマー間隔で実行する部分
	if tm.getFlagVal() > 0 % タイマーフラグをチェック
		tm.setFlagVal(0); % フラグおろす
		i = i +1; % カウンタをインクリメント

		% 画面表示用の設定
		[ data, time, s ] = mble.getReadData();	% BLEの受信メッセージを取得
		str = i +": "+ s + ", " + char(data) ; 	% 文字列の整形
		disp(str); % 画面表示

		% ループ回数の途中でメッセージ送信（BLE通信）
		switch i
			case 3	% 3秒後に
				mble.sendMessage('c'); % キャリブレーションコマンド
			case 5 % 5秒後に
				mble.sendMessage('a'); % arm状態コマンド
			case 7 % 7秒後に
				% !! ↓のアイドリングコマンドを送信するとプロペラが回転するので注意 !!
				% mble.sendMessage('i'); % アイドリングコマンド
			case 8 % 8秒後に
				% !! ↓の制御開始コマンドを送信するとプロペラが回転するので注意 !!
				% mble.sendMessage('s'); % 制御開始コマンド
		end
	end

	% アプリ終了判定
	if app.getQuitFlag() > 0  break; end

	pause(0.0001);	% 一時停止
end

mble.sendMessage('d');	% 停止指令を送信
pause(0.5);	% 一時停止

close gcf; % 図の終了

unsubscribe(mble.chara_read) % データ受信の購読を解除
clear mble	 % BLE通信のインスタンスを削除

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
