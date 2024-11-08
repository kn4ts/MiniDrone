function output = showplot( depth )
	
	% 引数がなければ最新ファイルを対象にする
	if nargin < 1
		depth = 0;
	end

	% インデックスの指定
	colm_time = 4 ; % マイコン時間[ms]
	colm_atti = [ 5, 7 ]; % 姿勢角
	colm_atti_f = [ 8, 10 ]; % 姿勢角
	colm_alti = [ 11, 12 ]; % 高度とフィルタ後の高度
	colm_uc = [  13, 16 ]; % 制御器出力
	colm_cf = [ 17, 20 ]; % 制御力
	colm_ref = [ 21, 23 ];
	colm_mode = 24 ; % モード
	colm_arm  = 25 ; % アーム状態
 

	folderPath = './output/shaped/'; % 参照データのフォルダの指定

	% csvファイル名を取得
	filelist = dir([ folderPath, '*.csv']);	

	fileNames = {filelist.name}; % ファイル名をセル配列として取得
	fileNames = string( fileNames ) ; % 文字列型に返還

	fileName = folderPath + fileNames(end -depth); % 抽出するファイル名を合成

	% データ読み込み
	data = readmatrix( fileName );

	% マイコン時間を計算
	time = data(:,colm_time) * 0.001 ; % [s]

	% モードが0でなくなる最初のインデックスを調べる
	index = find( data(:,colm_mode) ~= 0, 1);

	% ===================================================================================================
	%  図の設定
	% ===================================================================================================
	set(groot, 'DefaultAxesFontSize', 14);  % 軸のフォントサイズを14に設定
	set(groot, 'DefaultTextFontSize', 16);  % タイトルやラベルのフォントサイズを14に設定
	set(groot, 'DefaultLineLineWidth', 2);  % 線幅を2に設定
	set(groot, 'DefaultAxesFontName', 'TimesNewRoman');  % フォントの種類
	% ===================================================================================================
	figure()
	% プロット作成
	subplot( 2,2,1 )
	hold on
	plot( time(index:end), data( index:end, colm_alti(1) ) ); % 高度
	plot( time(index:end), data( index:end, colm_alti(2) ) ); % 高度のフィルタ値
	plot( time(index:end), data( index:end, colm_ref(1) ), 'r' ); % 高度の目標値
	hold off
	legend(["altitude [mm]", "altitude (filtered) [mm]", "reference [mm]"])
	ylabel("Altitude [mm]")
	box on, grid on
	xlim([ time(index), time(end) ])
	ax = gca;
	ax.XAxis.Exponent = 0;  % X軸の指数表示を無効に
	%
	subplot( 2,2,3 )
	hold on
	plot( time(index:end), data( index:end, colm_atti(1):colm_atti(2) ) ); % 姿勢角
	plot( time(index:end), data( index:end, colm_atti_f(1):colm_atti_f(2) ), '--' ); % 姿勢角のフィルタ値
	plot( time(index:end), data( index:end, colm_ref(2)-1:colm_ref(2) ), 'r:' ); % 姿勢角の目標値
	hold off
	legend(["roll [deg]", "pitch [deg]", "yaw [deg]", ...
	        "roll(filtered) [deg]", "pitch(filtered) [deg]", "yaw(filtered) [deg]", ...
			"reference(roll) [deg]", "reference(pitch) [deg]"])
	box on, grid on
	xlim([ time(index), time(end) ])
	ylabel("Attitude [deg]")
	xlabel("Time (in FC) [s]")
	ax = gca;
	ax.XAxis.Exponent = 0;  % X軸の指数表示を無効に
	%plot( data(:,colm_time), data(:,colm_alti(1):colm_alti(2)) )
	%
	subplot( 2,2,2 )
	plot( time(index:end), data( index:end, colm_uc(1):colm_uc(2) ) ); % 高度
	legend(["uc_1", "uc_2", "uc_3", "uc_4"])
	ylabel("Controller output [-]")
	box on, grid on
	xlim([ time(index), time(end) ])
	ax = gca;
	ax.XAxis.Exponent = 0;  % X軸の指数表示を無効に
	%
	subplot( 2,2,4 )
	plot( time(index:end), data( index:end, colm_cf(1):colm_cf(2) ) ); % 姿勢角
	legend(["tau_roll", "tau_pitch", "tau_yaw", "f_all"])
	box on, grid on
	xlim([ time(index), time(end) ])
	ylabel("Control force [-]")
	xlabel("Time (in FC) [s]")
	ax = gca;
	ax.XAxis.Exponent = 0;  % X軸の指数表示を無効に
	
	figure()
	subplot(2,1,1)
	plot( time(index:end), data( index:end, end-1:end ) ); % 時刻
	legend(["in PC time", "in FC time"])
	box on, grid on
	xlim([ time(index), time(end) ])
	ylabel("difference time [ms]")
	ax = gca;
	ax.XAxis.Exponent = 0;  % X軸の指数表示を無効に
	%
	subplot(2,1,2)
	plot( time(index:end), data( index:end, end-1:end ) ); % 時刻
	legend(["in PC time", "in FC time"])
	box on, grid on
	xlim([ time(index), time(end) ])
	ylim([ -1, 4 * max( median( data(index:end, end-1)), median( data(index:end, end))) ])
	ylabel("difference time [ms]")
	xlabel("Time (in FC) [s]")
	ax = gca;
	ax.XAxis.Exponent = 0;  % X軸の指数表示を無効に


	%output = data;
	%output = data(:,colm_time);
	%output = data(:,colm_atti(1):colm_atti(2));
	output = fileName ;
end
