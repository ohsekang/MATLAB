%%
% mobile 연동
m = mobiledev
orient = m.Orientation(1) % degrees
position = [0 0]; % start point
distance = 0.077; % 1 cycle 0.077m
turn_point = {[7 0] [7 12] [-7 12] [-7 0]} % curve point
sampleTime = 0.54;
vizRate = rateControl(1/sampleTime);

scout = canChannel('PEAK-System', 'PCAN_USBBUS1');
start(scout)
message = receive(scout, Inf, "OutputFormat", "timetable")

TxMsg = canMessage(1057, false, 1);
TxMsg.Data = [1];
transmit(scout,TxMsg)

TxMsg_linear = canMessage(273, false, 8);
TxMsg_linear.Data = ([0 150 0 0 0 0 0 0]);

TxMsg_rotate = canMessage(273, false, 8);
TxMsg_rotate.Data = ([0 0 0 200 0 0 0 0]);

% theta = initialorient(1) - currorient(1)
% % 왼쪽 회전 값이 작아짐, 오른쪽 회전 값이 커짐
% 
% position = position + [distance*cos(theta) distance*sin(theta)]

%%
orient = m.Orientation(1) + 180 % 0~360 범위로 변경
% 180 90 360 270
rotate_angle = [-90 -180 90 0] + 180 % 회전 후 바라보는 방향의 각도

%%
% 경로 추정
for i=1:4 % 다음 구간 진입
    while norm(turn_point{i}-position) > 0.1 % 직선 구간
        [xx, yy] = line1to4(distance, i);
        position = position + [xx, yy] % 현재 위치 갱신
        transmit(scout,TxMsg_linear)
        waitfor(vizRate);
    end

    % 회전에 대한 식 필요
    while orient > rotate_angle(i)
        orient = m.Orientation(1) + 180
        transmit(scout,TxMsg_rotate)
        waitfor(vizRate);
    end
end
