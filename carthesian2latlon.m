function [lat2_d, lon2_d] = carthesian2latlon(x, y, lat1_in, lon1_in)

    if size(lat1_in,2) > 1
        % Convert dms to degrees
        lat1_d = lat1_in(1) + lat1_in(2)/60 + lat1_in(3)/3600;
        lon1_d = lon1_in(1) + lon1_in(2)/60 + lon1_in(3)/3600;
    else
        lat1_d = lat1_in;
        lon1_d = lon1_in;
    end
    
    % Convert to radians
    lat1 = lat1_d*pi/180;
    lon1 = lon1_d*pi/180;
    
    radius = 6371e3; %Earth's radius

    deltaLat = y/radius;
    lat2 = deltaLat + lat1;

    deltaLon = x/(radius*cos((lat1+lat2)/2));
    lon2 = deltaLon + lon1;

    lat2_d = lat2*180/pi;
    lon2_d = lon2*180/pi;

end

