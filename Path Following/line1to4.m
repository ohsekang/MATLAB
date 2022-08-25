function [xx, yy] = line1to4(distance, i)
switch i
    case 1
        xx=distance;
        yy=0;
    case 2
        xx=0;
        yy=distance;
    case 3
        xx=-distance;
        yy=0;
    case 4
        xx=0;
        yy=-distance;
end