%%
clear; clc;

% 사용할 수 있는 can channel 검색
canChannelList

%%

% can 객체 생성
scout = canChannel('PEAK-System', 'PCAN_USBBUS1');

% 검색 가능하도록 온라인 설정
start(scout)

% can port를 통해 들어오는 데이터를 타임테이블 형태로 저장
message = receive(scout, Inf, "OutputFormat", "timetable")

%%

% Table 3.5 Control Mode Setting Frame
% scout-mini에 데이터를 전달하기 위한 포트 설정 및 활성화
TxMsg = canMessage(1057, false, 1);
TxMsg.Data = 1;
transmit(scout,TxMsg)

%%

% linear 0.15m/s
% data sheet에 맞게 변환
v = 0.15;
vv = typecast(swapbytes(int16(v*1000)), "uint8");

% scout-mini에 데이터 전달
TxMsgs = canMessage(273, false, 8);
TxMsgs.Data = ([vv 0 0 0 0 0 0]);

% 10초 동안 속도 전송
tic
while toc < 10
    transmit(scout,TxMsgs);
end
