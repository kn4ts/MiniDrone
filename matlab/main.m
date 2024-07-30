clc, close all, clear all

addpath('MatlabBLE\')	% BLE通信用クラスのパスを追加
addpath('DataFile\')	% データロガー用クラスのパスを追加
addpath('DataHandle\')	% データハンドルクラスのパスを追加

% データロガーの設定
OUTPUT_FOLDER = "./output/"; % データロガーの出力用フォルダを指定
df = DataFile( OUTPUT_FOLDER ) % データロガークラスのインスタンス生成

% BLE通信の設定
ID = "8DFC031CAF32"; % Bluetooth MAC アドレス
mble = MatlabBLE( ID )	% BLE通信のインスタンス生成

f = genCallbackFunction( df ); % BLE受信により起動させるコールバック関数を生成
mble.chara_read.DataAvailableFcn = @(src,evt) f( src, evt, ...
	mble.data, mble.time, mble.snum ); % コールバック関数の関連付け

N = 50;	% ループ回数を設定

%=======================
%	メインループ
%=======================
for i=1:N

	[ data, time, s ] = mble.getReadData();	% BLEの受信メッセージを取得

	% 画面表示用の設定
	str = s + ", " + char(data)+ ", " + time ;
	disp(str);
	
	% ループ回数の半分を超えたところでメッセージ送信（テスト）
	if( i>0.5*N )
		mble.sendMessage('0');	% BLE通信でメッセージ送信
	end

	pause(0.5);	% 一時停止
end

unsubscribe(mble.chara_read)

%=======================
%	関数定義
%=======================
% BLE受信コールバック関数を生成する高階関数
function f = genCallbackFunction( df )
	
	% コールバック関数の定義
	function callback( src, evt, da, ti, se )
		% 読み込んだデータを変数daに記録
		da.setVal( read( src ) );
		% PC時刻を変数tiに記録
		ti.setVal( MatlabBLE.getDateTimeString() );
		% データの通し番号をインクリメント
		se.setVal( se.getVal() +1 );

		%str = [ se.getVal(), ti.getVal(), char(da.getVal()) ];
		%str = [ se.getVal(), ti.getVal(), da.getVal() ];
		%str = [ num2str(se.getVal()), ',', char(ti.getVal()), ',', char(da.getVal()) ];
		str = [ num2str(se.getVal()), ',', char(ti.getVal()), ',', char(da.getVal()) ];

		%df.outputDataStr(se.getVal()+','+ti.getVal()+','+char(da.getVal()));
		df.outputDataStr( str );
	end
	f = @callback ;
end
