function readimuinf()
%     out = cell(10);
    a = arduino('COM4', 'Uno');
    imu = mpu6050(a);

    while 1
%         for i=1:5
% %         imu.release
% %         imu.flush
%         %         out{i} = readAcceleration(imu);
%         %         labSend(out{i}, 2);
% 
%         labSend(readAcceleration(imu), 2);
%         end
        labSend(readAcceleration(imu), 2);
        pause(0.1)
    end
end