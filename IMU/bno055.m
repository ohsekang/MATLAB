% serial port 14
clear; clc;
imu = serialport("COM14", 115200);
%%
flush(imu)
% readline(a)
tic
i=1;
while toc<1
readline(imu)
i=i+1
end

%%
Pre_data = readline(imu);
Data = str2num(Pre_data)

%%
time = linspace(1,10000,10000)';
Data = zeros(10000, 3);

%%
flush(imu)
figure
tic
while(toc < 60)
    Pre_data = readline(imu);
    Data(1,:) = str2num(Pre_data);
    plot(time, Data(:,1), '.')
    drawnow limitrate
    Data = circshift(Data, 1);
end

%%
figure
plot(Data(:, 1))