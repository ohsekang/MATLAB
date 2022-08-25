TxMsg_linear = canMessage(273, false, 8);
TxMsg_linear.Data = ([0 150 0 0 0 0 0 0]);
tic
while toc<5
    transmit(scout,TxMsg_linear)
    rplidar
end