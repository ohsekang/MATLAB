%%
% ros2 연결 확인

setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
getenv("RMW_IMPLEMENTATION");

setenv('ROS_DOMAIN_ID','0')
getenv('ROS_DOMAIN_ID');

test1 = ros2node('/test1');
test2 = ros2node('/test2');

ros2 node list
ros2 topic list -t
ros2 msg list

%%
% lidar topic 연결하여 lidar data 저장
ridarSub = ros2subscriber(test1, "/rplidar_ros/scan");
tic
figure
i=1;
scans = struct([]);

while toc < 330
    ridarData = receive(ridarSub,2);
    scan = rosReadLidarScan(ridarData);
    scans{i} = scan;
    rosPlot(ridarData)
    i = i+1;
end

%%
% camera topic과 연결
cameraSub = ros2subscriber(test2, "/camera_pkg/display_mjpeg");

[scanData,status,statustext] = receive(cameraSub,5);

% image 크기 변환
sc = permute(reshape(scanData.data, [480 120]), [2 1]);
sc1 = permute(reshape(permute(scanData.data, [2 1]), [160 120 3]), [2 1 3]);

% 알맞은 이미지 크기 비교
figure
subplot(221); imshow(sc)
subplot(222); imshow(sc1(:,:,1))
subplot(223); imshow(sc1(:,:,2))
subplot(224); imshow(sc1(:,:,3))

%%
% video 영상 받아오기
cameraSub1 = ros2subscriber(test2, "/camera_pkg/video_mjpeg");

%%
tic
[scanData1,status,statustext] = receive(cameraSub1,5)

sc = permute(reshape(scanData.data, [480 120]), [2 1]);

figure, imshow(sc)
toc

%%
% cameraSub1 = ros2subscriber(test2, "/camera/aligned_depth_to_color/image_raw");
cameraSub2 = ros2subscriber(test2, "/camera/color/image_raw");
% cameraSub3 = ros2subscriber(test2, "/camera/depth/image_rect_raw");

tic
% [scanData1,status1,statustext1] = receive(cameraSub1,5);
[scanData2,status2,statustext2] = receive(cameraSub2,5);
% [scanData3,status3,statustext3] = receive(cameraSub3,5);

sc = permute(reshape(scanData2.data, [3840 720]), [2 1]);
imshow(sc)
toc
