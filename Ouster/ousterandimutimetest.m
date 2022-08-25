%%
% lidar + imu 받아오기

clear; clc;

spmd(2)

    if labindex == 1 % lidar
        ousterobj = ousterlidar("OS1-64", "CurrentSensorCalibration.json");
        sampleTime = 0.25;
        vizRate = rateControl(1/sampleTime);

        pClouds = cell(1000, 1);
        j=1;
        tic
        while toc < 20
            pClouds{j} = read(ousterobj, "latest");
            j=j+1;
            if j==11
                toc
            end
            waitfor(vizRate);
        end
    
    elseif labindex == 2 % imu
        sampleTime = 0.25;
        vizRate = rateControl(1/sampleTime);

%         time = linspace(1,10000,10000)';
        Data = zeros(1000, 3);

        imu = serialport("COM14", 115200);
        flush(imu)
        j=1;
        tic
        while toc < 20
            Data(j,:) = str2num(readline(imu));
            j=j+1;
            waitfor(vizRate);
        end
    end
end
%%
j{1}, j{2}