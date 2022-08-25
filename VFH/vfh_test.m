%%
% 가로, 세로 넉넉히 0.6m -> l = 0.8485 = 0.4243 * 2
vfh = controllerVFH;
vfh.UseLidarScan = true; % lidarScan data를 입력으로 사용
vfh.DistanceLimits = [0.3 1]; % 범위 값 : 최대/최소
vfh.RobotRadius = 0.4243; % 차량 반경(r)
vfh.SafetyDistance = 0.5; % 차량 안전거리
vfh.MinTurningRadius = 0.5; % 현재속도에서 최소 회전 반경(외부 점 기준 회전) *** 수정 필요
vfh.TargetDirectionWeight = 5; % 목표 방향에 대한 비용 함수 가중치
vfh.CurrentDirectionWeight = 2; % 값이 높을수록 효율적인 경로 생성
vfh.PreviousDirectionWeight = 3; % 값이 높을수록 부드러운 경로 생성
vfh.HistogramThresholds;

targetDir = 0;
sampleTime = 0.545; % 0.545;
vizRate = rateControl(1/sampleTime);

while 1
    tic
	% Get laser scan data
	rplidar;
    
	% Call VFH object to computer steering direction
	steerDir = vfh(scan, targetDir);  
    % 차량의 조향 방향(전진 방향부터 반시계 방향으로 x라디안으로 값 증가)
    
	% Calculate velocities
	if ~isnan(steerDir) % 직진해도 되는 경우 (객체 미검출)
		disp('go')
	else % Stop and search for valid direction
		disp('stop')
    end
    
    waitfor(vizRate);
end

%%
% 가로, 세로 넉넉히 0.6m -> l = 0.8485 = 0.4243 * 2
vfh = controllerVFH;
vfh.UseLidarScan = true; % lidarScan data를 입력으로 사용
vfh.DistanceLimits = [0.3 1]; % 범위 값 : 최대/최소
vfh.RobotRadius = 0.4243; % 차량 반경(r)
vfh.SafetyDistance = 0.5; % 차량 안전거리
vfh.MinTurningRadius = 0.5; % 현재속도에서 최소 회전 반경(외부 점 기준 회전) *** 수정 필요
vfh.TargetDirectionWeight = 5; % 목표 방향에 대한 비용 함수 가중치
vfh.CurrentDirectionWeight = 2; % 값이 높을수록 효율적인 경로 생성
vfh.PreviousDirectionWeight = 3; % 값이 높을수록 부드러운 경로 생성
vfh.HistogramThresholds;

targetDir = 0;
sampleTime = 0.545; % 0.545;
vizRate = rateControl(1/sampleTime);

while 1
%     tic
    
    while toc < 0.5
        % Get laser scan data
        rplidar;

        % Call VFH object to computer steering direction
        steerDir = vfh(scan, targetDir);
        % 차량의 조향 방향(전진 방향부터 반시계 방향으로 x라디안으로 값 증가)

        % Calculate velocities
        if ~isnan(steerDir) % 직진해도 되는 경우 (객체 미검출)
            disp('go')
        else % Stop and search for valid direction
            disp('stop')
        end
        %     flag = rplidar_flag(pRPLIDAR, vfh)
%         toc
        %     flag
    end
    waitfor(vizRate);
end
%%
vfh = controllerVFH;
vfh.UseLidarScan = true; % lidarScan data를 입력으로 사용
vfh.DistanceLimits = [0.2 0.5]; % 범위 값 : 최대/최소
vfh.RobotRadius = 0.4243; % 차량 반경(r)
vfh.SafetyDistance = 0.5; % 차량 안전거리
vfh.MinTurningRadius = 0.5; % 현재속도에서 최소 회전 반경(외부 점 기준 회전) *** 수정 필요
vfh.TargetDirectionWeight = 5; % 목표 방향에 대한 비용 함수 가중치
vfh.CurrentDirectionWeight = 2; % 값이 높을수록 효율적인 경로 생성
vfh.PreviousDirectionWeight = 3; % 값이 높을수록 부드러운 경로 생성
vfh.HistogramThresholds;

while 1
    rplidar;

    % Call VFH object to computer steering direction
    steerDir = vfh(scan, targetDir);
    % 차량의 조향 방향(전진 방향부터 반시계 방향으로 x라디안으로 값 증가)

    % Calculate velocities
    if ~isnan(steerDir) % 직진해도 되는 경우 (객체 미검출)
        disp('go')
    else % Stop and search for valid direction
        disp('stop')
    end
    waitfor(vizRate);
end
%%
while 1
    tic
    rplidar;
    if flag == 1
        disp('stop')
    else
        disp('go')
    end
    toc
end
%%
while 1
    tic
    while toc < 0.4
        rplidar;
        if flag == 1
            break
        end
    end
    toc
end