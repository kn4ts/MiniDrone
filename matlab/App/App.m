%% ==============================================
%%  App のクラス定義
%%  ...キー割り込み機能の実装
%%				2024/07/12
%% 				K.N
%% ==============================================
classdef App
	properties
		Flag_rec	% 記録フラグ
		Flag_quit	% 終了フラグ

		Flag_read	% キー割り込みフラグ
		cha 	% 読み込んだ文字

		fig	% キー割り込み用の図のインスタンス
	end
	methods
		% コンストラクタ
		function obj = App()
			obj.Flag_rec  = DataHandle(0) ; % フラグの初期化
			obj.Flag_quit = DataHandle(0) ; % フラグの初期化
			%obj.fig = DataHandle();
			obj.cha = DataHandle('0'); % 取得文字の初期化
			obj.Flag_read = DataHandle(0); % フラグの初期化
			
			% コールバック用のfigureを生成
			obj.fig = figure('position',[0 0 eps eps],'menubar','none');
			% コールバック関数を紐づけ
			obj.fig.KeyPressFcn = @(src,data) ...
				obj.callback(src, data, obj.Flag_rec, obj.Flag_quit, obj.Flag_read, obj.cha) ;
			%disp("MATLAB : Accept key interrupt")
		end
		%% 図の終了メソッド
		%function quitFigure(obj)
		%	close obj.fig ;
		%end
		% 記録判定フラグのゲッタ
		function val = getRecFlag(obj)
			val = obj.Flag_rec.val;
		end
		% 終了判定フラグのゲッタ
		function val = getQuitFlag(obj)
			val = obj.Flag_quit.val;
		end

		% 読み込みフラグのゲッタ
		function val = getReadFlag(obj)
			val = obj.Flag_read.val;
		end
		% 読み込みフラグのセッタ
		function setReadFlag(obj, x)
			obj.Flag_read.val = x;
		end
		% 文字のゲッタ
		function cha = getReadChara(obj)
			cha = obj.cha.val;
		end
	end
	methods(Static)
		% キー割り込み関数
		function callback( src, data, Flag_rec, Flag_quit, Flag_read, cha )

			Flag_read.val = 1;
			cha.val = data.Key ;
			
			% 記録開始判定
			%if strcmp( data.Key, 'r' )
			%	%disp("Key Pressed!")
			%	Flag_rec.val = 1;
			%end
		
			%% 記録終了判定
			%if strcmp( data.Key, 'e' )
			%	% 記録フラグを下ろす
			%	Flag_rec.val = 0;
			%end

			%% プログラム終了判定
			if strcmp( data.Key, 'q' )
				% 終了フラグを上げる
				Flag_quit.val = 1;
				% 
				disp("プログラム終了")
			end
		end
	end
end
