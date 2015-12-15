function u = points2vectU(p1,p2)
    dx = p2(1) - p1(1);
    dy = p2(2) - p1(2);
    norm = sqrt(dx^2+dy^2);
    u = [dx/norm; dy/norm];
end