%% Data Preprocessing
clc; 
% load data2.mat
% ptClouds = LIDAR(615:-1:2);
% insData = IMU(615:-1:2, :);
insData = IMU(615:-1:2, :);
insData(303:end,1) = insData(303:end,1) - 360;

%% data preprocessing  - 0811 12F
ptClouds = pc(1:585, 1);
insData = ori(1:585, :);
idx = insData(:, 1) > 300;
insData(idx, 1) = insData(idx, 1) - 360;

%% Point Cloud filter with roi
pClouds_Filtered=cell(height(ptClouds),1);
for i = 1:height(ptClouds)
    indices = findPointsInROI(ptClouds{i},[-12 12 -12 12 -0.75 0.75]);
    % [-3 3 -3 3 -0.75 0.75] or [-6 6 -6 6 -0.75 0.75]
    pClouds_Filtered{i} = select(ptClouds{i},indices);
end

%% Point Cloud vs Point Cloud filter with roi
figure; pcshow(ptClouds{1})
figure; pcshow(pClouds_Filtered{1})

%% Reshape Point Cloud Data format
horizontalResolution=1024;
params = lidarParameters('OS2-64',horizontalResolution);
ptCloudOrgs=cell(height(ptClouds),1);
for i=1:height(ptClouds)
    ptCloudOrgs{i} = pcorganize(pClouds_Filtered{i},params);  
end
%% Point Cloud downsampling
% random downsampling과 grid filter downsampling(모양 유지에 유리) 둘 다 해보고 비교
% grid step 클수록 점 갯수 줄어듦(?)
rng(0);
%gridStep = 0.1;
downSamplePercentage=0.25;
pClouds_Downsampled=cell(height(ptClouds),1);
for i=1:height(pClouds_Filtered)
    pc=pClouds_Filtered{i};
    %pClouds_Downsampled{i}=pcdownsample(pClouds_Filtered{i},'gridAverage',gridStep);
    pClouds_Downsampled{i}=pcdownsample(pc,'random',downSamplePercentage);
end

%% Point Cloud Registration (relative pose)
gridStep=1;
pClouds_registered=cell(height(pClouds_Downsampled),1);
insData=cell(height(pClouds_Downsampled),1);
for i=1:2:height(pClouds_Downsampled)
    if i==1
        pClouds_registered{i}=pClouds_Downsampled{i};
        tform=[0 0 0 0 0 0 0];
        relPose = [0 0 0 0 0 0 0];
        insData{i}=relPose;
    else
    moving=pClouds_Downsampled{i};
    fixed=pClouds_Downsampled{i-1};
    tform=pcregisterndt(moving,fixed,gridStep);
    relPose = [tform2trvec(tform.T') tform2quat(tform.T')];
    insData{i}=relPose;
    pClouds_registered{i}=pctransform(moving,tform);
    end
end

%% Point Cloud PCView for pose graph(NDT)
vSet = pcviewset;

absPose = rigid3d;
relPose = rigid3d;

vSet = addView(vSet,1,absPose,'PointCloud',pClouds_Downsampled{1});
insData=cell(height(pClouds_Downsampled),1);
skipFrames = 2;
prevViewId = 1;
prevPtCloud = pClouds_Downsampled{1};
insData{1}=[0 0 0];
%i=1;

for viewId = 1+skipFrames:skipFrames:height(pClouds_Downsampled)
    
    ptCloud_map=pClouds_Downsampled{viewId};
    % Register new point cloud against the previous one.
    regGridStep = 1.6; %1
    relPose = pcregisterndt(ptCloud_map,prevPtCloud,regGridStep, ...
        'InitialTransform',relPose);

    % Update the absolute transform.
    absPose = rigid3d(relPose.T*absPose.T);
    insData{viewId}=[tform2trvec(relPose.T') tform2quat(relPose.T')];
    % Add new view and connection to the previous view.
    vSet = addView(vSet,viewId,absPose,'PointCloud',ptCloud_map);
    vSet = addConnection(vSet,prevViewId,viewId,relPose);

    prevPtCloud = ptCloud_map;
    prevViewId = viewId;
end

figure(5);
plot(vSet,'ShowViewIds','on')
view(2)
%% Point Cloud PCView for pose graph(ICP)

vSet = pcviewset;

absPose = rigid3d;
relPose = rigid3d;

vSet = addView(vSet,1,absPose,'PointCloud',pClouds_Downsampled{1});

skipFrames = 5;
prevViewId = 1;
prevPtCloud = pClouds_Downsampled{1};

for viewId = 6:skipFrames:height(pClouds_Downsampled)
    
    ptCloud_map=pClouds_Downsampled{viewId};
    % Register new point cloud against the previous one.
    relPose = pcregistericp(ptCloud_map,prevPtCloud,'Extrapolate',true,'InitialTransform',relPose,...
        'Metric','pointToPoint');

    % Update the absolute transform.
    absPose = rigid3d(relPose.T*absPose.T);

    % Add new view and connection to the previous view.
    vSet = addView(vSet,viewId,absPose,'PointCloud',ptCloud_map);
    vSet = addConnection(vSet,prevViewId,viewId,relPose);

    prevPtCloud = ptCloud_map;
    prevViewId = viewId;
end

figure(2);
plot(vSet,'ShowViewIds','on')
view(2)

%% Point Cloud PCView for pose graph(corr) 
vSet = pcviewset;

absPose = rigid3d;
relPose = rigid3d;

vSet = addView(vSet,1,absPose,'PointCloud',pClouds_Downsampled{1});

skipFrames = 5;
prevViewId = 1;
prevPtCloud = pClouds_Downsampled{1};

for viewId = 6:skipFrames:height(pClouds_Downsampled)
    
    ptCloud_map=pClouds_Downsampled{viewId};
    % Register new point cloud against the previous one.
    relPose = pcregistercorr(ptCloud_map,prevPtCloud,100,0.5);

    % Update the absolute transform.
    absPose = rigid3d(relPose.T*absPose.T);

    % Add new view and connection to the previous view.
    vSet = addView(vSet,viewId,absPose,'PointCloud',ptCloud_map);
    vSet = addConnection(vSet,prevViewId,viewId,relPose);

    prevPtCloud = ptCloud_map;
    prevViewId = viewId;
end

figure(3);
plot(vSet,'ShowViewIds','on')
view(2)
%% Point Cloud PCView for pose graph(loam) 
vSet = pcviewset;

absPose = rigid3d;
relPose = rigid3d;

vSet = addView(vSet,1,absPose,'PointCloud',pClouds_Downsampled{1});

skipFrames = 5;
prevViewId = 1;
prevPtCloud = ptClouds{1};

for viewId = 6:skipFrames:height(pClouds_Downsampled)
    
    ptCloud_map=ptClouds{viewId};
    % Register new point cloud against the previous one.
    relPose = pcregisterloam(ptCloud_map,prevPtCloud,1);

    % Update the absolute transform.
    absPose = rigid3d(relPose.T*absPose.T);

    % Add new view and connection to the previous view.
    vSet = addView(vSet,viewId,absPose,'PointCloud',ptCloud_map);
    vSet = addConnection(vSet,prevViewId,viewId,relPose);

    prevPtCloud = ptCloud_map;
    prevViewId = viewId;
end

figure(4);
plot(vSet,'ShowViewIds','on')
view(2)
%% test
player=pcplayer([-8 8],[-8 8],[-5 5]);

for i=1:2:height(ptClouds)
     view(player,ptClouds{i});
%     view(player,pClouds_Filtered{i});
%     view(player,pClouds_Downsampled{i});
%     view(player,pClouds_registered{i});
%     view(player,ptCloudOrgs{i});
     pause(0.1);
end

%% build map 예제
clc; rng(0);

% Create a lidar map builder
mapBuilder = helperLidarMapBuilder('DownsamplePercent', 0.75,'MergeGridStep', 0.4, ...
     'RegistrationGridStep', 1, 'Verbose', true); 
% 0.8 / 0.2 / 0.9
% 0.7 0.2 0.85
% 0.7 0.1 0.98 good

% Configure the map builder to detect loop closures
configureLoopDetector(mapBuilder, ...
    'LoopConfirmationRMSE',  2, ...%2
    'SearchRadius',          0.1, ...%0.15
    'DistanceThreshold',     0.07);%0.07 or 0.02

% Loop through the point cloud array and progressively build a map
exitLoop = false;

skipFrames = 2;
Tform=rigid3d;
prevInsMeas = insData(1, :);
for n = 1:skipFrames:height(ptClouds)

    insMeas = insData(n, :);

    % Estimate initial transformation using INS
    initTform = helperEstimateRelativeTransformationFromINS(insMeas, prevInsMeas);
    %initTform=Tform;
    % Update map with new lidar frame
    Tform=updateMap(mapBuilder, pClouds_Filtered{n}, initTform);%ptClouds

    % Update top-view display
    isDisplayOpen = updateDisplay(mapBuilder, exitLoop);

    % Check and exit if needed
    exitLoop = ~isDisplayOpen;

    prevInsMeas = insMeas;
end

snapnow;

%% build map 예제 - quat
clc; rng(0);

% Create a lidar map builder
mapBuilder = helperLidarMapBuilder('DownsamplePercent', 0.75,'MergeGridStep', 0.4, ...
     'RegistrationGridStep', 1, 'Verbose', true); 
% 0.8 / 0.2 / 0.9
% 0.7 0.2 0.85
% 0.7 0.1 0.98 good

% Configure the map builder to detect loop closures
configureLoopDetector(mapBuilder, ...
    'LoopConfirmationRMSE',  2, ...%2
    'SearchRadius',          0.1, ...%0.15
    'DistanceThreshold',     0.07);%0.07 or 0.02

% Loop through the point cloud array and progressively build a map
exitLoop = false;


Tform=rigid3d;
prevInsMeas = insData(1, :);
for n = 2:height(ptClouds)

    insMeas = insData(n, :);

    % Estimate initial transformation using INS
%     initTform = helperEstimateRelativeTransformationFromINS(insMeas, prevInsMeas);
    initTform = IMUsensorFusion(PrevImuData,CurImuData,initTform);

    %initTform=Tform;
    % Update map with new lidar frame
    Tform=updateMap(mapBuilder, pClouds_Filtered{n}, initTform);%ptClouds

    % Update top-view display
    isDisplayOpen = updateDisplay(mapBuilder, exitLoop);

    % Check and exit if needed
    exitLoop = ~isDisplayOpen;

    prevInsMeas = insMeas;
end

snapnow;
%% 
sMap = pcmapsegmatch('CentroidDistance',1);
outerCylinderRadius = 30;
innerCylinderRadius = 3;
distThreshold = 0.5;
angleThreshold = 180;

for n = 1:height(ptClouds)
    ptCloud_map = ptClouds{n};

    % Segment and remove the ground plane.
    indices = findPointsInROI(pointCloud(pClouds{i}),[-3 3 -3 3 -0.75 0.75]);%[-3 3 -3 3 -0.75 0.75]
    pClouds_Filtered{i} = select(ptCloud_map,indices);

    % Select cylindrical neighborhood.
    dists = sqrt(ptCloud_map.Location(:,:,1).^2 + ptCloud_map.Location(:,:,2).^2);
    cylinderIdx = dists <= outerCylinderRadius ...
        & dists > innerCylinderRadius;
    ptCloud_map = select(ptCloud_map,cylinderIdx,'OutputSize','full');

    % Segment the point cloud.
    [labels, numClusters] = segmentLidarData(ptCloud_map,distThreshold,angleThreshold,'NumClusterPoints',[50 5000]);

    % Extract features from the point cloud.
    [features,segments] = extractEigenFeatures(ptCloud_map,labels);

    % Add the features and segments to the map.
    sMap = addView(sMap,n,features,segments);
end

figure; show(sMap);


%% build 3d occupancy map
map3D = occupancyMap3D(10);
maxRange=10;
for i=1:2:height(pClouds_Filtered)
    pose=insData{i};
    points=pClouds_Downsampled{i}.Location;
    insertPointCloud(map3D,pose,points,maxRange);
end    
show(map3D)

%%
% Set the occupancy grid size to 100 m with a resolution of 0.2 m
ptCloudProcessed=mapBuilder.Map;
% Set the occupancy grid size to 100 m with a resolution of 0.2 m
gridSize = 35;
gridStep = 0.1;

% The occupancy grid is created by scaling the points from 1m - 5m in
% height to the probability values of [0 1]
zLimits = [0 3];%[1 5]

sensorHeight=0.8;

locationPts = ptCloudProcessed.Location;
locationPts(:,3) = locationPts(:,3) + sensorHeight;
ptCloudHeightAdjusted = pointCloud(locationPts);
ptCloudHeightAdjusted=pcdenoise(ptCloudHeightAdjusted,"NumNeighbors",1,"Threshold",10);

% Calclate the number of bins
%spatialLimits = [-gridSize/2 gridSize/2; -gridSize/2 gridSize/2; ptCloudHeightAdjusted.ZLimits];
spatialLimits = [-gridSize/2 gridSize/2; -gridSize/2 gridSize/2; ptCloudProcessed.ZLimits];

gridXBinSize = round(abs(spatialLimits(1,2) - spatialLimits(1,1)) / gridStep);
gridYBinSize = round(abs(spatialLimits(2,2) - spatialLimits(2,1)) / gridStep);

numBins = [gridXBinSize gridYBinSize 1];

% Find bin indices
%binIndices = pcbin(ptCloudHeightAdjusted,numBins,spatialLimits,'BinOutput',false);
binIndices = pcbin(ptCloudProcessed,numBins,spatialLimits,'BinOutput',false);


% Pre allocate occupancy grid
% occupancyGrid = zeros(gridXBinSize,gridYBinSize,'like',ptCloudHeightAdjusted.Location);
occupancyGrid = zeros(gridXBinSize,gridYBinSize,'like',ptCloudProcessed.Location);
gridCount = zeros(gridXBinSize,gridYBinSize);

% Scale the Z values of the points to the probability range [0 1]
%zValues = rescale(ptCloudHeightAdjusted.Location(:,3),'InputMin', zLimits(1),'InputMax', zLimits(2));
zValues = rescale(ptCloudProcessed.Location(:,3),'InputMin', zLimits(1),'InputMax', zLimits(2));

for idx = 1:numel(binIndices)
    binIdx = binIndices(idx);
    if ~isnan(binIdx)
        occupancyGrid(binIdx) = occupancyGrid(binIdx) + zValues(idx);
        gridCount(binIdx) = gridCount(binIdx) + 1;
    end
end

gridCount(gridCount == 0) = 1;

occupancyGrid = occupancyGrid ./ gridCount;

% Visualize the created occupancy grid
figure;
subplot(1,2,1);
pcshow(ptCloudProcessed); view(2);
title('Point cloud birds eye view')
subplot(1,2,2);
imshow(imrotate(occupancyGrid, 90)); 
tHandle = title('Occupancy grid image');
tHandle.Color = [1 1 1];
%%
testMap=occupancyMap(occupancyGrid,1);
figure;
show(testMap)

%%
function initTform = IMUsensorFusion(PrevImuData,CurImuData,initTform)
rotation = eulerd(PrevImuData*conj(CurImuData), 'ZYX','point') * -1;
% remove pitch and roll using only yaw
rotation = quaternion([rotation(1) 0 0],'eulerd','ZYX');
initTform.Rotation = rotmat(rotation,'point');
end