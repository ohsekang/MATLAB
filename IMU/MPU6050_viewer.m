%%
% https://kr.mathworks.com/help/supportpkg/arduinoio/ug/estimating-orientation-using-inertial-sensor-fusion-and-mpu9250.html?searchHighlight=HelperOrientationViewer&s_tid=srchtitle_HelperOrientationViewer_2

% https://kr.mathworks.com/help/supportpkg/arduinoio/ref/mpu6050-system-object.html
%%
clear; clc;
ar = arduino('COM4', 'Uno');
imu = mpu6050(ar);

%%

% From mu*g / sqrt(Hz) to m/s^2
psd = 400*1e-6;
ome = 2*pi*10;  % f is in Hz(I believe it is 10 Hz according to the MPU6050 datasheet).
psd2ToAcc = psd*sqrt(ome); % Converting psd to acceleration unit 'g'
AccNoise = convacc(psd2ToAcc, 'G''s', 'm/s^2');

% From (deg/s)/sqrt(Hz) to rad/s
nsd = 0.005;
ome = 2*pi*10;
nsd2ToAcc = nsd*sqrt(ome);
GyroNoise = deg2rad(nsd2ToAcc);

% GyroNoise = deg2rad(0.05)^2;


viewer = HelperOrientationViewer('Title',{'IMU Filter'});
% FUSE = imufilter('GyroscopeNoise',GyroNoise^2,'AccelerometerNoise',AccNoise^2);
FUSE = imufilter('GyroscopeNoise',GyroNoise,'AccelerometerNoise',AccNoise);


for i=1:1000
    
%     acc = readAcceleration(imu);
%     gyro = readAngularVelocity(imu);
%     rotators = FUSE(acc, gyro);
    flush(imu)
    data = read(imu);
    rotators = FUSE(data.Acceleration, data.AngularVelocity);
    for j = numel(rotators)
        viewer(rotators(j));
    end
end