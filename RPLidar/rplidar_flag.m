function flag = rplidar_flag(pRPLIDAR, vfh)
    a = 360*2;
    count = 0; alldistances = zeros([1 a]); allangles = zeros([1 a]);
    targetDir = 0;
    
    flag = 0;
    while (count <= a)
        [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
    
        % +-30도에 해당하는 값만 받아옴
        if -0.5236 <= angles && angles <= 0.5236
            alldistances(count+1) = distances;
            allangles(count+1) = angles;
        end
    
        %if bNewScan
        if count == a
            scan = lidarScan(alldistances, allangles);
            steerDir = vfh(scan, targetDir);
            if isnan(steerDir) % 직진해도 되는 경우 (객체 미검출)
                flag = 1;
            end
        end
        count = count+1;
    end

end