%%
% data 불러오기
load data19_F4.mat scans

%%

scans = scans(1:710); % scans의 값들을 전부 사용해도 되지만 일부만 사용해서 더 좋은 결과를 내기도 하기에 인덱싱하여 다양하게 수행

maxRange = 5.51; % 라이다의 최대 인식 거리, 소숫점 사용가능 (m, 미터단위)
resolution = 11; % grid map 한칸 당 실제 거리, 소숫점 사용불가 (m, 미터단위)

slamObj = lidarSLAM(resolution,maxRange); % SLAM 객체 생성
slamObj.LoopClosureThreshold = 200; % 한 바퀴 돌았을 때의 임계점, 높을수록 우수한 일치를 보이지만 보통 센서 데이터에 따라 달라짐
slamObj.LoopClosureSearchRadius = 8; % 한 바퀴 돌았다는 것을 인지하는 반경, 높을수록 탐지 시간이 길어짐
slamObj.LoopClosureMaxAttempts = 1; % 한 바퀴 돌았다는 것을 인지하기 위해 수행하는 연산의 수, 높을수록 탐지 시간이 길어짐

for i = 1:numel(scans1) 

    addScan(slamObj,scans1{i});
    
    if rem(i,10) == 0
        show(slamObj);
    end
end

[scansSLAM,poses] = scansAndPoses(slamObj); % Map과 로봇이 이동했던 위치들을 그림으로 표현
deepracerMap = buildMap(scansSLAM,poses,resolution,maxRange);
figure
show(deepracerMap)
title('Occupancy Map of deepracer')
