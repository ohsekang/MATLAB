%%
clear;clc
delete(gcp())
p = parpool(2);
%%
clear; clc;
Array_IMUxLIDAR = parfeval(@Read_Data,2);

%%
clear; clc;
a = serialport("COM14", 115200);
% lidar 연결
lidar = ousterlidar("OS1-64","CurrentSensorCalibration.json");
time = linspace(1,1000,1000)';
IMU = zeros(1000, 3);
LIDAR = cell(1000,1);
flush(a)
tic
while(toc < 200)
    LIDAR{1} = read(lidar,'latest');
    LIDAR = circshift(LIDAR,1);
    Pre_data = readline(a);
    IMU(1,:) = str2num(Pre_data);
    IMU = circshift(IMU, 1);
end

%%
tic
% LIDAR{1} = read(lidar,'latest');
% LIDAR = circshift(LIDAR,1);
IMU(1,:) = str2num(Pre_data);
IMU = circshift(IMU, 1);
toc
%%
function [IMU,LIDAR] = Read_Data
        % imu 연결
        a = serialport("COM14", 115200);
        % lidar 연결
        lidar = ousterlidar("OS1-64","CurrentSensorCalibration.json");
        time = linspace(1,3000,3000)';
        IMU = zeros(3000, 3);
        LIDAR = cell(3000,1);
        flush(a)
        tic
        while(toc < 8)
            Pre_data = readline(a);
            IMU(1,:) = str2num(Pre_data);
            IMU = circshift(IMU, 1);
            LIDAR{1} = read(lidar,'latest');
            LIDAR = circshift(LIDAR,1);
        end
end