function robotCurrentPose = local_purepersuit_TurnOrient(local_controller, robot, scout, distanceTonNighborPoint, robotCurrentPose, local_path, TxMsgs)
    sampleTime = 0.54; % 0.545;
    vizRate = rateControl(1/sampleTime);
    goalRadius = 0.2;

    % 가까운 포인트까지 pure persuit로 이동
    while distanceTonNighborPoint > goalRadius
    
        % Compute the controller outputs, i.e., the inputs to the robot
        [vl, omegal] = local_controller(robotCurrentPose);
    
        % Get the robot's velocity using controller inputs
        vell = derivative(robot, robotCurrentPose, [vl omegal]);
    
        % Update the current pose
        robotCurrentPose = robotCurrentPose + vell*sampleTime;
    
        % Re-compute the distance to the goal
        distanceTonNighborPoint = norm(robotCurrentPose(1:2,:) - local_path(end,:));
    
        vv = typecast(swapbytes(int16(vl*1000)), "uint8");
        ome = typecast(swapbytes(int16(omegal*1000)), "uint8");
    
        TxMsgs.Data = ([vv ome 0 0 0 0]);
        transmit(scout,TxMsgs)
        waitfor(vizRate);
    end

    % 가까운 포인트까지 이동이 끝나면 바라보는 각도를 수정
    % 이 작업 대신 계속 다음 경로 따라가도록 path planning 하기로 함
end