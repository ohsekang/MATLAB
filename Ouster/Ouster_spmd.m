
%%
% lidar 연결
clear; clc;
ousterobj = ousterlidar("OS1-64", "CurrentSensorCalibration.json");

%%
% lidar 1초에 7개 받아옴
tic
j=1;
while toc < 1
    pClouds{j} = read(ousterobj, "latest");
    j=j+1;
end

%%
% 병렬 처리
parpool(2);

%%
% lidar + imu 받아오기

clear; clc;

spmd(2)

    if labindex == 1 % lidar
        ousterobj = ousterlidar("OS1-64", "CurrentSensorCalibration.json");
        sampleTime = 0.25;
        vizRate = rateControl(1/sampleTime);

        pClouds = cell(1000, 1);
        tic
        for j=1:1000
            pClouds{j} = read(ousterobj, "latest");
            if j==12
%                 toc
                disp(1)
            end
            waitfor(vizRate);
        end
    
    elseif labindex == 2 % imu
        sampleTime = 0.25;
        vizRate = rateControl(1/sampleTime);

%         time = linspace(1,10000,10000)';
        Data = zeros(1000, 3);

        imu = serialport("COM14", 115200);
        flush(imu);
        
        for j=1:1000
            Data(j,:) = str2num(readline(imu));
            waitfor(vizRate);
        end
    end
end

%%

j{1}, j{2}