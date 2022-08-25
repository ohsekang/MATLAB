parpool(4)
%% 
clear; clc;


spmd(4)
    if labindex == 1 % 1번 작업공간: imu 데이터 받기
        Array = zeros(10,9);
        
        a = arduino('COM4','Uno','Libraries','I2C');
        imu =  mpu6050(a,'OutputFormat','matrix');
        
        for i=1:10
        accel = readAcceleration(imu);
        gyro = readAngularVelocity(imu);
%         mag = readMagneticField(imu);
        
        Array(1, 1:3) = accel;
        Array(1, 4:6) = gyro;
%         Array(1, 7:9) = mag;

        labSend(Array(1,:),2); % 2번 작업공간으로 데이터 전송
        
        Array = circshift(Array,1);
        end
    
    elseif labindex == 2 % 2번 작업공간 imu 특정값 저장 
      %  accel x값 
      tic
      data = zeros(10,9);
        for i=1:10
            X = labReceive("any");
            if X(1,1) >= 5.4
                data(1,:) = X;
                data = circshift(data,1);
            end
            toc
       end

    elseif labindex == 3
        if not(libisloaded('hardwarex'))
            addpath('.');
            switch (computer)
                case 'PCWIN64'
                    addpath('x64');
                    %loadlibrary('hardwarex', @hardwarex_proto);
                    [notfound,warnings]=loadlibrary('hardwarex', @hardwarex_proto);
                case 'PCWIN'
                    addpath('x86');
                    %loadlibrary('hardwarex');
                    [notfound,warnings]=loadlibrary('hardwarex', @hardwarex_proto);
                otherwise
                    %loadlibrary('hardwarex');
                    [notfound,warnings]=loadlibrary('hardwarex', 'hardwarex.h', 'includepath', 'MAVLinkSDK', 'includepath', '/usr/local/include', 'includepath', 'sbgECom/src', 'includepath', 'sbgECom/common', 'includepath', '/usr/local/include/sbgECom/src', 'includepath', '/usr/local/include/sbgECom/common', 'includepath', 'rplidar_sdk/sdk/sdk/include', 'includepath', 'rplidar_sdk/sdk/sdk/src', 'includepath', '/usr/local/include/rplidar_sdk/sdk/sdk/include', 'includepath', '/usr/local/include/rplidar_sdk/sdk/sdk/src');
            end
            %libfunctions hardwarex -full
        end

        pRPLIDAR = calllib('hardwarex', 'CreateRPLIDARx');

        result = calllib('hardwarex', 'ConnectRPLIDARx', pRPLIDAR, 'RPLIDAR0.txt');

        a = 360;
        count = 0;

        distances = zeros(360,1);
        angles = zeros(360,1);

        while (count < a)
            [~, distance, angle, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
            count = count+1;
            distances(count) = distance;
            angles(count) = angle;
        end

        labSend([distances, angles], 4);
        disp(result)
    else
        X = labReceive("any");
    end
end