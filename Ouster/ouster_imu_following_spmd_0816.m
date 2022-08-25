%%
% 병렬 처리
parpool(2);

%% 12F

% lidar + imu 받아오기
pause(5);
clear; clc;

spmd(2)

    if labindex == 1 % lidar + imu
        ousterobj = ousterlidar("OS1-64", "CurrentSensorCalibration.json");
        imu = serialport("COM3", 115200);

        pClouds = cell(1200, 1);
        Data = zeros(1200, 3);
        j=1;
        flush(imu);

        tic
        while toc < 300
            pClouds{j} = read(ousterobj, "latest");
            Data(j,:) = str2num(readline(imu));
            j=j+1;
        end

    elseif labindex == 2 % 주행
        % data load
        data = load('v_lin_ome1.mat');
%         data = load('v_lin_ome2.mat');
        sampleTime = 0.545;
        vizRate = rateControl(1/sampleTime);
        % can 통신
        scout = canChannel('PEAK-System', 'PCAN_USBBUS1');
        start(scout);
        % 전송 모드 설정
        TxMsg = canMessage(1057, false, 1);
        TxMsg.Data = [1];
        transmit(scout,TxMsg)
        TxMsgs = canMessage(273, false, 8);
        %         labSend(1, 1); % 1번 작업공간으로 데이터 전송
        % 주행
        for k=1:308
            %             disp(data.v_lin_ome(k, :))
            TxMsgs.Data = ([data.v_lin_ome(k, :) 0 0 0 0]);
            transmit(scout, TxMsgs);
            waitfor(vizRate);
        end
    end
end

%% 4, 5, 7, 8F

% lidar + imu 받아오기
pause(3);
clear; clc;
disp('start')
spmd(2)

    if labindex == 1 % lidar + imu
        ousterobj = ousterlidar("OS1-64", "CurrentSensorCalibration.json");
        imu = serialport("COM3", 115200);

        pClouds = cell(1200, 1);
        Data = zeros(1200, 3);
        j=1;
        flush(imu);

        tic
        while toc < 300
            pClouds{j} = read(ousterobj, "latest");
            Data(j,:) = str2num(readline(imu));
            j=j+1;
        end
    end
end
%%
j = j{1} -1;
pc = pClouds{1}; % cell
pc = pc(1:j, 1);
ori = Data{1}; % matrix
ori = ori(1:j, :);

%%
save("lidar_imu_8F_220816.mat", "pc", "ori");
