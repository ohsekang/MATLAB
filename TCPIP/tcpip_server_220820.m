%% 
% tcp ip server 생성
server = tcpserver("192.168.0.49", 80); % server ip 주소 입력

%% 
% client에 전달하고 싶은 데이터 작성
write(server,[6,9,14,26,27,42],"uint8") % 원하는 데이터와 데이터 타입 입력