parpool(2);
%%
clear; clc;
spmd(2)
    if labindex == 1 % 주행
        % data load
        data = load('v_lin_ome1.mat');
        %         v_lin_ome = load('v_lin_ome2.mat');
        sampleTime = 0.545;
        vizRate = rateControl(1/sampleTime);
        % can 통신
        scout = canChannel('PEAK-System', 'PCAN_USBBUS1');
        start(scout);
        % 전송 모드 설정
        TxMsg = canMessage(1057, false, 1);
        TxMsg.Data = [1];
        transmit(scout,TxMsg)
        TxMsg = canMessage(273, false, 8);
%         labSend(1, 1); % 1번 작업공간으로 데이터 전송
        % 주행
        for k=1:308
%             disp(data.v_lin_ome(k, :))
            TxMsg.Data = ([data.v_lin_ome(k, :) 0 0 0 0]);
            transmit(scout, TxMsg);
            waitfor(vizRate);
        end
    elseif labindex == 2 % 주행
        disp(1)
    end
end