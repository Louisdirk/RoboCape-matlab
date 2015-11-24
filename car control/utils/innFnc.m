function inn = innFnc(t,z,y,maxP,maxT,tStart)

if length(z)==2 % GPS
    if t>tStart
        inn = [min(max(z(1:2)-y(1:2),-maxP),maxP);];
    else
        inn = [z(1:2)-y(1:2)];
    end
elseif  length(z)==3 % GPS
    if t>tStart
        inn = [min(max(z(1:2)-y(1:2),-maxP),maxP);
            min(max(angleErrorRad(z(3),y(3)),-maxT),maxT)];
    else
        inn = [z(1:2)-y(1:2);
            angleErrorRad(z(3),y(3))];
    end
    
else
    inn    = z - y;
    inn(6) = angleErrorRad(z(6),y(6));
end
end