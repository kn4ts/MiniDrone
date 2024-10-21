%
% ファイル出力用データのクラス定義
%
%  DataHandle型の変数と，タイマー構造体を要素として持つ
%
classdef DataFile
	properties
		fn	% ファイル名を格納
	end
	methods
		% コンストラクタ
		%  引数1： fn ... 保存ファイル名
		function obj = DataFile( folder_name )
			% フォーマット定義
			fmt = 'yyyyMMdd_HHmmss';
			% DataHandle型の変数を初期化
			obj.fn = folder_name + "exp_" +string( datetime( 'now', 'Format', fmt ) ) +".csv";
			%obj.fn = DataHandle( 0 ); 
		end
		% ファイルへの書き出し関数
		function outputDataStr( obj, str )
			writematrix( str, obj.fn, 'WriteMode', 'append');
		end
	end
	methods(Static)
		% 記録用文字列整形関数
		function str = genStrWrite( cnt, seTime, seIntv, mag, y, r, u, ua )
			str = [ cnt, ... 	% 制御ループのカウンタ
				seTime, ... 	% シリアル割り込み時刻
				seIntv, ... 	% シリアル割り込みの間隔
				mag', ...	% 発光強度（ベクトル）
				r', ... 	% 制御の目標値
				y', ... 	% 制御出力
				u', ... 	% 制御入力1（流量指令値）
				ua' ]; 		% 実際の流量
		end
	end
end
