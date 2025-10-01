%% ==============================================
%%  Hori RAP のクラス定義
%%				2025/09/20
%% 				K.N
%% ==============================================
classdef HoriRap
    properties
        joy % vrjoystickオブジェクト

        joyID   % ジョイスティックID

        axes    % 軸の値
        buttons % ボタンの値
        povs    % POV（十字キー）の値
                %  POVの値は以下の通り
                %  -1 : 中立
                %   0 : 下
                %  45 : 左下
                %  90 : 左
                % 135 : 左上
                % 180 : 上
                % 225 : 右上
                % 270 : 右
                % 315 : 右下
        pre_povs % 前回のPOVの値

        %flagPOVchanged % POVの状態が変化したかどうかのフラグ
    end

    methods
        % HoriRapクラスのコンストラクタ
        function obj = HoriRap()
            ids = [];
            for k = 1:8
                try
                    obj.joy = vrjoystick(k);
                    ids(end+1) = k; fprintf("Joystick %d 検出\n", k);
                catch
                    fprintf( "Joystick %d 未検出\n", k );
                end
            end
            if isempty(ids)
                disp("ジョイスティックが検出されませんでした");
                obj = HoriRap.empty ; % 空のオブジェクトを返す
                return
            else
                obj.joyID = DataHandle( ids(1) );

                % ジョイスティックの特性取得
                [ a, b, p ] = read( obj.joy ) ;
                numAxes = length(a);     % 軸の数
                numButtons = length(b);  % ボタンの数
                numPOVs = length(p);     % POVの数

                obj.axes    = DataHandle( zeros(1, numAxes) );
                obj.buttons = DataHandle( zeros(1, numButtons) );
                obj.povs    = DataHandle( zeros(1, numPOVs) );

                obj.pre_povs = DataHandle( zeros(1, numPOVs) );
            end
            %obj.flagPOVchanged = DataHandle( false );
        end

        % ジョイスティックの状態を読み取り→プロパティに格納
        function updateJoyState(obj)
            try
                [axes, buttons, povs] = read( obj.joy ) ;
                obj.axes.setVal(axes);
                obj.buttons.setVal(buttons);

                obj.pre_povs.setVal( obj.povs.getVal() ); % 前回のPOV状態を保存
                obj.povs.setVal(povs);
            catch
                error("ジョイスティック読み取りエラー");
            end
        end

        % ジョイスティックのゲッタ
        function joy = getJoy(obj) , joy = obj.joy; end
        % ジョイスティックIDのゲッタ
        function id = getJoyID(obj) , id = obj.joyID.getVal(); end

        % 軸、ボタン、POVのゲッタ
        function axes = getAxes(obj) , axes = obj.axes.getVal(); end
        function buttons = getButtons(obj), buttons = obj.buttons.getVal(); end
        function povs = getPOVs(obj) , povs = obj.povs.getVal(); end
        
        function pov_Xdirection = getPOVXdirection(obj)
            pov_Xdirection = obj.getXprojection( obj.getPOVs() ) ;
        end

        % POVの状態が変化したかどうかをチェックするメソッド
        function flag = checkPOVsChanged( obj )
            %persistent prevPOVs % 前回の状態を保存する変数
            prevPOVs = obj.pre_povs.getVal();
            currPOVs = obj.povs.getVal();

            if ~isequal(prevPOVs, currPOVs)
                flag = true;
            else
                flag = false;
            end
            %obj.flagPOVchanged.setVal(flag);
        end

        % POVのx方向成分状態が変化したかどうかをチェックするメソッド
        function flag = checkPOVsChangedXdirection( obj )
            %persistent prevPOVs % 前回の状態を保存する変数
            prevPOVs_X = obj.getXprojection( obj.pre_povs.getVal() );
            currPOVs_X = obj.getXprojection( obj.povs.getVal() );

            if ~isequal( prevPOVs_X, currPOVs_X )
                flag = true;
            else
                flag = false;
            end
            %obj.flagPOVchanged.setVal(flag);
        end

    end
    methods (Static)
        % POVの値からx方向の符号を計算するヘルパーメソッド
        function x = getXprojection( pov )
            if isempty(pov)
                x = 0;
                return;
            end

            % POVの値に基づいて横方向の射影を計算
            switch pov
                case -1 % 中立
                    x = 0;
                case 0 % 下
                    x = 0;
                case 45 % 左下
                    x = -1;
                case 90 % 左
                    x = -1;
                case 135 % 左上
                    x = -1;
                case 180 % 上
                    x = 0;
                case 225 % 右上
                    x = 1;
                case 270 % 右
                    x = 1;
                case 315 % 右下
                    x = 1;
                otherwise
                    x = 0; % 不明な値の場合は中立とみなす
            end
        end
    end
end