%% tcp ip client 생성
client = tcpclient("192.168.0.49", 80); % 본인(client) ip 주소 입력

%% server에서 데이터 받아오기
read(client, client.NumBytesAvailable) % 읽을 수 있는 데이터 전체 읽어오기