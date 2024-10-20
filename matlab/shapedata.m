function out = shapedata()
	folderPath = './output/'; % 整形元データのフォルダの指定
	shapedFolderPath = 'shaped/';	% 出力先フォルダの指定

	% csvファイル名を取得
	filelist_origin = dir([ folderPath, '*.csv']);	
	filelist_output = dir([ folderPath, shapedFolderPath, '*.csv']);	
	%filelist = dir('./output/*.csv');

	fileNames_A = {filelist_origin.name}; % ファイル名をセル配列として取得
	% fileNames_A = string( fileNames_A ); % ファイル名をString型に変換

	fileNames_B = {filelist_output.name}; % ファイル名をセル配列として取得
	% fileNames_B = string( fileNames_B ); % ファイル名をString型に変換

	fileNames = setdiff( fileNames_A, fileNames_B ); % 差集合を取る
	fileNames = string( fileNames ) ;
	%name = {filelist.name};
	%N = filelist.name
	%extension = 'csv' ;
	N = length( fileNames ); % ファイルの数を取得

	%searchPattern = fullfile(folderPath, ['*.' extension]);
	%files = dir( searchPattern );

	disp("整形対象のファイル数 ... " + N)

	colm_time = 3 ; % マイコン時間[ms]

    if( N > 0 )
	    % ファイルの数の分ループ
	    for i=1:N
		    % セル配列でファイル読み込み
		    content = readcell( string(folderPath) + fileNames(i) );
    
		    % 1行が1かたまりの文字列になっていれば（処理対象として）if文の中へ
		    if ( size( content, 2 ) < 2 )
			    % Stringベクトルに変換
			    content = string( content );
			    % 区切り記号でString配列に変換
			    strmat = split( content, ',' );
			    % 配列のサイズを取得
			    [ Nx, Ny ] = size( strmat ) ;
			    
			    % 数値読み込みのための行列を用意
			    Mat = zeros( Nx, Ny );
    
			    % 各要素毎にループ
			    for j = 1:Nx % 行方向
				    for k = 1:Ny % 列方向
					    % 数値に変換して行列に格納
					    Mat(j,k) = str2num(strmat(j,k));
				    end
			    end

			    % マイコン時間の差分を計算して追加
			    Tm = diff( Mat(:,colm_time) );
			    Tm( Nx, 1 ) = 0;
			    Mat = [ Mat, Tm ];
    
			    % 
			    writematrix( Mat, ...
				    string(folderPath) + string(shapedFolderPath) + fileNames(i), ...
				    'WriteMode', 'overwrite' );
		    end
        end
    else
        Mat = "No file to shape";
    end
	
	%out = fileNames ;
	%out = readcell(string(folderPath)+fileNames(13)) ;
	%out = readcell(string(folderPath)+fileNames(13)) ;
	out = Mat;
end
