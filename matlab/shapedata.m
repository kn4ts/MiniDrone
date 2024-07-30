function out = shapedata()
	folderPath = './output/';
	shapedFolderPath = 'shaped/';
	filelist = dir([ folderPath, '*.csv']);
	%filelist = dir('./output/*.csv');

	fileNames = {filelist.name}; % ファイル名をセル配列として取得
	fileNames = string( fileNames );
	%name = {filelist.name};
	%N = filelist.name
	%extension = 'csv' ;
	N = length( fileNames );

	%searchPattern = fullfile(folderPath, ['*.' extension]);
	%files = dir( searchPattern );
	for i=1:N
		% セル配列で読み込み
		content = readcell( string(folderPath) + fileNames(i) );

		% 1行が1かたまりの文字列になっていればif文の中へ
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
			for j = 1:Nx
				for k = 1:Ny
					% 数値に変換して行列に格納
					Mat(j,k) = str2num(strmat(j,k));
				end
			end

			writematrix( Mat, ...
				string(folderPath) + string(shapedFolderPath) + fileNames(i), ...
				'WriteMode', 'overwrite' );
		end
	end
	
	%out = fileNames ;
	%out = readcell(string(folderPath)+fileNames(13)) ;
	%out = readcell(string(folderPath)+fileNames(13)) ;
	out = Mat;
end
