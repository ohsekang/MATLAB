%%
% 병렬 처리
parpool(2);

%%
% 리모컨으로 lidar + imu 받아오기
pause(5);

clear; clc;

spmd(2)

    if labindex == 1
        ousterobj = ousterlidar("OS1-64", "CurrentSensorCalibration.json");
        imu = serialport("COM14", 115200);

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

%% 데이터 가공
j = j{1};
pc = pClouds{1}; % cell
pc = pc(1:j, 1);
ori = Data{1}; % matrix
ori = ori(1:j, :);

%%

pc1 = pc(1:100);
pc2 = pc(101:200);
pc3 = pc(201:300);
pc4 = pc(301:400);
pc5 = pc(401:500);
pc6 = pc(501:600);
pc7 = pc(601:700);
pc8 = pc(701:800);
pc9 = pc(801:900);
pc10 = pc(901:1000);

%%

save("lidar_imu_0812_4F_1.mat", "pc1");
save("lidar_imu_0812_4F_2.mat", "pc2");
save("lidar_imu_0812_4F_3.mat", "pc3");
save("lidar_imu_0812_4F_4.mat", "pc4");
save("lidar_imu_0812_4F_5.mat", "pc5");
save("lidar_imu_0812_4F_6.mat", "pc6");
save("lidar_imu_0812_4F_7.mat", "pc7");
save("lidar_imu_0812_4F_8.mat", "pc8");
save("lidar_imu_0812_4F_9.mat", "pc9");
save("lidar_imu_0812_4F_10.mat", "pc10");