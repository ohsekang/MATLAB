%% load data
clc;
% load data1.mat
ori = ori(1:585,:);
pc = pc(1:1:585);
%% adjust
clc;
% for i=1:height(ori)
%     if(ori(i,1)>300)
%         ori(i,1)=ori(i,1)-360;
%     end
% end
% Point Cloud filter with roi
pClouds_Filtered=cell(height(pc),1);
tic
for i = 1:height(pc)
indices = findPointsInROI(pc{i},[-20 20 -20 20 -0.5 0.5]);%[-5 5 -5 5 -0.75 0.75]
pClouds_Filtered{i} = select(pc{i},indices);
end
toc
%%
%rotation 구할 때 quaternion으로 들어가야 됨
ori = quaternion(eul2quat(deg2rad(ori)));
%% build map
clc;
rng(0);
% Create a lidar map builder
mapBuilder = helperLidarMapBuilder('DownsamplePercent', 0.7,'MergeGridStep',0.3, ...
     'RegistrationGridStep', 1.5, 'Verbose', true); % 0.7 0.3 1.5 good
% Configure the map builder to detect loop closures
configureLoopDetector(mapBuilder, ...
    'LoopConfirmationRMSE',  0.26, ...%2
    'SearchRadius',          1, ...%0.15
    'DistanceThreshold',     0.07);%0.07 or 0.02
% Loop through the point cloud array and progressively build a map
exitLoop   = false;
initTform = rigid3d;
%prevInsMeas = imuData{1,:};%insData{1, :};%imu
i=1;
for n = 2:3:height(pc)
    insMeas = quat2eul(ori(n,:));
    % Estimate initial transformation using INS
%     initTform = helperEstimateRelativeTransformationFromINS(insMeas, prevInsMeas);%imu
    rotation = eulerd(ori(n-1)*conj(ori(n)),'ZYX','point');
    % remove pitch and roll using only yaw
    rotation(1,2:3) =  0;
    r = quaternion(rotation,'eulerd','ZYX','point');
    initTform.T(1:3,1:3) = rotmat(r,'point');
    % Update map with new lidar frame
    initTform = updateMap(mapBuilder, pClouds_Filtered{n}, initTform);%ptClouds
    % Update top-view display
    isDisplayOpen = updateDisplay(mapBuilder, exitLoop);
    % Check and exit if needed
    exitLoop = ~isDisplayOpen;
    prevInsMeas = insMeas;
    i=i+1;
end
snapnow;
%% build 3d occupancy map
map3D = occupancyMap3D(10);
maxRange=10;
for i=1:2:height(pClouds_Filtered)
    pose=insData{i};
    points=pClouds_Downsampled{i}.Location;
    insertPointCloud(map3D,pose,points,maxRange);
end
show(map3D)