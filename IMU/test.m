out = Composite;
% tic
spmd(2)
    if labindex == 1
        readimuinf();
    else
%         tic
%         while toc < 1
%             out = labReceive('any');
%             disp(out)
%         end
        out = labReceive('any');
        disp(out)
    end
end
% toc

%%
cnt = 0
tic
while toc < 1
    cnt = cnt + 1;
%     flush(imu);
    data = read(imu)
    
    spmd(2)
        if labindex == 1
            labSend(data, 2);
        else
            out = labReceive('any');
            disp(out)
        end
    end
end

disp(cnt)
%%
out = Composite;

tic
while toc < 1
    spmd(2)
        if labindex == 1
            getobj(imu);
        else
            out = labReceive('any');
            disp(out)
        end
    end
end