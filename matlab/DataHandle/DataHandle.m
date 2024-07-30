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

		function setVal( obj, value )
			obj.val = value;
		end
		function val = getVal( obj )
			val = obj.val ;
		end
	end
end
