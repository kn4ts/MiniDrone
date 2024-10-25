%% ==============================================
%%  Matlab BLE のクラス定義
%%				2024/07/12
%% 				K.N
%% ==============================================
classdef MatlabBLE
	properties
		dev	% デバイス情報
		chara_read	% 特性
		chara_send	% 特性
		N_max = 5 % 最大接続試行回数の定義

		data	% データハンドル（参照渡し対応）
		time	% データハンドル（参照渡し対応）
		snum	% データハンドル（参照渡し対応）
	end
	methods
		% BLEクラスのコンストラクタメソッド
		function obj = MatlabBLE( name ) 

			% BLEデバイスへの接続部分
			for i=1:obj.N_max
				disp("Connecting to -> " + name)
				% センサへの接続を試行
				try
					obj.dev = ble( name );     % センサデバイスの宣言
					%disp("Connection success (Polar H10): " + name)
					disp("       ... success")
					break
				% エラーが出たら接続を再試行
				catch ME
					% 再試行回数が最大に達していなければ接続を再試行
					if i>obj.N_max-1
						disp("       ... failed")
						%return
					% 再試行回数が最大数N_maxに達していれば，接続を断念
					else
						disp("       ... failed ( Retry to connect )")
					end
					continue
				end
			end
			% センサのデータ格納用ハンドルの初期化
			obj.data = DataHandle( 0 );
			obj.time = DataHandle( obj.getDateTimeString() );
			obj.snum = DataHandle( 0 );

			if ~isempty( obj.dev )
				% デバイスの特性（キャラクタリスティック）を操作するためのインスタンスを生成
				obj.chara_read = characteristic( obj.dev, ...
					"7E57283A-6D54-4C1B-9B19-1F0C438A81BC", ...
					"1F9B4EA7-80B0-4C02-95B7-5C8E739A08F6" );	
				obj.chara_send = characteristic( obj.dev, ...
					"7E57283A-6D54-4C1B-9B19-1F0C438A81BC", ...
					"d2e5cbb1-7f7e-4d3d-93f6-792d7e0f70db" );	
			%	% 「デバイスからの通知」にコールバック関数を関連付ける
			%	% 　デバイスからの通知...デバイスがメッセージを書き込んだらPCに通知が来る機能
				% obj.chara_read.DataAvailableFcn = @(src,evt) MatlabBLE.callback(src, evt, ...
				% 	obj.data, obj.time, obj.snum );
			else
				disp("       ... Sensor was not found")
			end
		end

		%function [ hr, ts, se ] = getSensorData( obj )
		%	hr = obj.dh.hr ;
		%	ts = obj.dh.ts ;
		%	se = obj.dh.se ;
		%end
		function sendMessage( obj, str )
			write( obj.chara_send, uint8(str) );
		end
		% 読み込み関数関数
		function [ data, time, snum ] = getReadData( obj )
			data = obj.data.getVal() ;
			time = obj.time.getVal() ;
			snum = obj.snum.getVal() ;
		end

	end

	methods (Static)
		%% コールバック関数の定義
		function callback( src, evt, da, ti, se )
			% 計測値の読み出し
			%[ H10data, datetime ] = read( src, 'oldest' );
			da.setVal( read( src ) );
			% 割り込み時刻を取得
			ti.setVal( MatlabBLE.getDateTimeString() );
			% 読み込んだ計測値をパース
			%[ dh.hr, temp, err ] = PolarH10.translateHRandHRV( H10data );
			% データの通し番号をインクリメント
			%itr = se.getVal();
			se.setVal( se.getVal() +1 );

			%df.outputDataStr(da.getval());
		end

		function now = getDateTimeString()
			now = string(datetime, "HHmmss.SSSS");
		end
	end
end

