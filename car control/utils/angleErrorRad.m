function ret = angleErrorRad (z,y)
%z-y
z = mod(z,2*pi);
y = mod(y,2*pi);

err = [z-y,  z-y + 2*pi, z-y - 2*pi];

indexMin = err.*err==min(err.*err);
ret = err(indexMin);

end

function ret = angleErrorDeg (z,y)
%z-y
z = mod(z,360);
y = mod(y,360);

err = [z-y,  z-y + 360, z-y - 360];

indexMin = err.*err==min(err.*err);
ret = err(indexMin);

end
function testAngleError(z,y)

angleErrorDeg (180,-170) %-10
angleErrorDeg (-170,180) % 10
angleErrorDeg (5,-5)     % 10
angleErrorDeg (-5,+5)    % -10

end

