%%
% 병렬 처리
parpool(2);

%%
clear; clc;

spmd(2)

    if labindex == 1 % lidar + imu
        
        a = serialport("COM14", 115200); % imu 연결

        lidar = ousterlidar("OS1-64","CurrentSensorCalibration.json"); % lidar 연결
%         time = linspace(1,1000,1000)';
        IMU = zeros(1000, 3);
        LIDAR = cell(1000,1);
        
        flag = labReceive("any"); % 값을 받을 때 까지 대기
        
        flush(a)
        flush(lidar)

        tic
        while(toc < 200)
            LIDAR{1} = read(lidar,'latest');
            LIDAR = circshift(LIDAR,1);
            Pre_data = readline(a);
            IMU(1,:) = str2num(Pre_data); % str2num
            IMU = circshift(IMU, 1);
        end
    
    elseif labindex == 2 % 주행

        % data load
        v_lin_ome = load('v_lin_ome1.mat');
%         v_lin_ome = load('v_lin_ome2.mat');
        
        sampleTime = 0.545;
        vizRate = rateControl(1/sampleTime);

        % can 통신
        scout = canChannel('PEAK-System', 'PCAN_USBBUS1');

        start(scout);
        
        % 전송 모드 설정
        TxMsg = canMessage(1057, false, 1);
        TxMsg.Data = [1];
        transmit(scout,TxMsg)

        TxMsg = canMessage(273, false, 8);
        
        labSend(1, 1); % 1번 작업공간으로 데이터 전송

        % 주행
        for k=1:length(ome)
            TxMsg.Data = ([v_lin_ome(k) 0 0 0 0]);
            transmit(scout, TxMsg)
            waitfor(vizRate);
        end
    end
end
