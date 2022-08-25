function out=readimu()
    a = arduino('COM4', 'Uno');
    imu = mpu6050(a);
    out = read(imu);
    labSend(out, 2);
end