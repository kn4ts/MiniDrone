%% ===========================================================
%%  DataHandle のクラス定義
%%  ... handleクラスを継承した，クラスプロパティ用の変数クラス
%%						2024/07/12
%% 						K.N
%% ===========================================================
classdef DataHandle < handle
	properties
		val	% 格納する値
	end
	methods
		% コンストラクタ
		%  引数： data ... 値の初期値
		function obj = DataHandle( value )
			obj.setVal( value );
		end

		function setVal( obj, value ) % セッタ
			obj.val = value;
		end
		function val = getVal( obj ) % ゲッタ
			val = obj.val ;
		end
	end
end
