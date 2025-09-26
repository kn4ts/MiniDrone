%% ==============================================
%%  Timer のクラス定義
%%				2024/07/12
%% 				K.N
%% ==============================================
classdef Timer
	properties
		flag	% ハンドルを継承したデータ型

		t	% タイマー構造体
	end
	methods
		% コンストラクタ
		%  引数1： intvl ... インターバル[s]
		%  引数2： numloop  ... 繰り返し回数
		function obj = Timer( intvl, numloop )
			% DataHandle型の変数を初期化
			obj.flag = DataHandle( 0 ); 
			% タイマーインスタンスを生成
			obj.t = timer( ...
				'StartDelay', 1, ...	% タイマー開始ディレイ [s]
				'Period', intvl, ...	% タイマー実行間隔 [s]
				'TasksToExecute', numloop, ...	% タイマー実行回数
				'ExecutionMode', 'fixedRate' );	% タイマー実行モード
			% タイマー割り込み関数のセット
			obj.t.TimerFcn = { @obj.callback, obj.flag } ;
			% タイマースタート
			start(obj.t) ;
		end
		% フラグ値を取得するメソッド（ゲッタ）
		function val = getFlagVal( obj )
			val = obj.flag.val ;
		end
		% フラグ値を設定するメソッド（セッタ）
		function setFlagVal( obj, val )
			obj.flag.val = val ;
		end
	end
	methods(Static)
		% タイマー割り込み関数の定義
		function callback( src, event, flag )
			flag.val  = 1 ; % Flagを上げる
		end
	end
end
