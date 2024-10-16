function output = showplot( depth )
	
	% 引数がなければ最新ファイルを対象にする
	if nargin < 1
		depth = 0;
	end

	% インデックスの指定
	colm_time = 3 ; % マイコン時間[ms]
	colm_atti = [ 4, 6 ]; % 姿勢角
	colm_alti = 7; % 高度
	colm_uc = [  8, 11 ]; % 制御器出力
	colm_cf = [ 12, 15 ]; % 制御力
	colm_mode = 16 ; % モード
	colm_arm  = 17 ; % アーム状態
 

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

	figure()
	% プロット作成
	subplot( 2,2,1 )
	plot( time(index:end), data( index:end, colm_alti ) ); % 高度
	legend(["altitude [mm]"])
	ylabel("Altitude [mm]")
	box on, grid on
	xlim([ time(index), time(end) ])
	%
	subplot( 2,2,3 )
	plot( time(index:end), data( index:end, colm_atti(1):colm_atti(2) ) ); % 姿勢角
	legend(["roll [deg]", "pitch [deg]", "yaw [deg]"])
	box on, grid on
	xlim([ time(index), time(end) ])
	ylabel("Attitude [deg]")
	xlabel("Time [s]")
	%plot( data(:,colm_time), data(:,colm_alti(1):colm_alti(2)) )
	%
	subplot( 2,2,2 )
	plot( time(index:end), data( index:end, colm_uc(1):colm_uc(2) ) ); % 高度
	legend(["uc_1", "uc_2", "uc_3", "uc_4"])
	ylabel("Controller output [-]")
	box on, grid on
	xlim([ time(index), time(end) ])
	%
	subplot( 2,2,4 )
	plot( time(index:end), data( index:end, colm_cf(1):colm_cf(2) ) ); % 姿勢角
	legend(["tau_roll", "tau_pitch", "tau_yaw", "f_all"])
	box on, grid on
	xlim([ time(index), time(end) ])
	ylabel("Control force [-]")
	xlabel("Time [s]")
	
	%output = data;
	%output = data(:,colm_time);
	output = data(:,colm_atti(1):colm_atti(2));
end
