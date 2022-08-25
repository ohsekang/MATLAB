function num = quadrant(orient)
if 0 <= orient && orient <= pi/2
    num = 1;
elseif pi/2 <= orient && orient < pi
    num = 2;
elseif -pi < orient && orient <= -pi/2
    num = 3;
elseif -pi/2 <= orient && orient <= 0
    num = 4;
else
end