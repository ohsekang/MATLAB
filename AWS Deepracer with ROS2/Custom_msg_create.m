%%
% CMake MATLAB에 경로 추가
oldPath = getenv('PATH');
newPath = strcat(oldPath, pathsep, 'C:\Program Files\CMake\bin'); % on Windows
setenv('PATH', newPath);
system('cmake --version')

%%
% MATLAB에 Custom msg 빌드
folderPath = fullfile(pwd,"custom");
% copyfile("C:\Program Files\MATLAB\R2022a\examples\ros\data\example_*_msgs",folderPath);
ros2genmsg(folderPath)

%%
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

ros2 msg show sensor_msgs/Image

% ros2 msg show geometry_msgs/Pose

ros2 msg show deepracer_interfaces_pkg/ServoCtrlMsg

%% 
% deepracer에게 angle(바퀴 각도), throttle(속도) 만큼 속도 전송
% Pub = ros2publisher(test1, "/webserver_pkg/manual_drive")
Pub = ros2publisher(test1, "/ctrl_pkg/servo_msg")
msg = ros2message(Pub);
msg.angle = cast(0, 'single');
msg.throttle = cast(0, 'single');
% msg.MessageType = 'deepracer_interfaces_pkg/ServoCtrlMsg'
send(Pub, msg)
