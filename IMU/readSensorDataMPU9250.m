function [accel,gyro,mag] = readSensorDataMPU9250(imu)
% This function reads the imu sensor created using mpu9250 system object data from the 9 DOF sensor
% align the axis in accordance with NED coordinates and removes hard iron distortions
% Hard iron distortion correction values in units of microtesla
magx_correction = 2.85;
magy_correction = 27.2531;
magz_correction = -25.5820;
[accel,gyro,mag] = read(imu);
% Align coordinates in accordance with NED convention
accel = [-accel(:,2), -accel(:,1), accel(:,3)];
gyro = [gyro(:,2), gyro(:,1), -gyro(:,3)];
mag = [mag(:,1)-magx_correction, mag(:, 2)- magy_correction, mag(:,3)-magz_correction];
end

