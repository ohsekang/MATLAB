%%
% hardware 연결
% clear; clc;
hardwarex_init;
pRPLIDAR = CreateRPLIDAR();
% Check and modify the configuration file if needed...
[result] = ConnectRPLIDAR(pRPLIDAR, 'RPLIDAR0.txt')

%%

% ScanMode parameter in RPLIDAR0.txt might need to be changed depending on what is uncommented here...
[result, distance, angle, bNewScan, quality] = GetScanDataResponseRPLIDAR(pRPLIDAR);
str = sprintf('Distance at %f deg = %f m\n', angle*180.0/pi, distance);
disp(str);

fig = figure('Position',[200 200 400 400],'NumberTitle','off');
% Force the figure to have input focus (required to capture keys).
set(fig,'WindowStyle','Modal'); axis('off');
scale = 10;

scans = struct([]);
i=1;

count = 0;
alldistances = zeros([1 360]);
allangles = zeros([1 360]);
key = 0;

while (isempty(key)||(key ~= 27)) % Wait for ESC key (ASCII code 27).
    [result, distances, angles, bNewScan, quality] = GetScanDataResponseRPLIDAR(pRPLIDAR);   
    alldistances(count+1) = distances;
    allangles(count+1) = angles;

    if count > 360
        scan = lidarScan(alldistances, allangles);
        scans{i} = scan;
        clf; hold on; axis([-scale,scale,-scale,scale]);
        plot(alldistances.*cos(allangles), alldistances.*sin(allangles), '.');
        pause(0.01); key = get(gcf,'CurrentCharacter');
        count = 0; 
        alldistances = []; 
        allangles = [];
        i=i+1;
    end
    count = count+1;
end

%%
close(fig);

result = DisconnectRPLIDAR(pRPLIDAR);
DestroyRPLIDAR(pRPLIDAR);
clear pRPLIDAR; % unloadlibrary might fail if all the variables that use types from the library are not removed...
unloadlibrary('hardwarex');
