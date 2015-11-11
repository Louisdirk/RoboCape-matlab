function [x, y] = latlon2carthesian(lat1_dms, lon1_dms, lat2_dms, lon2_dms)

% Computes carthesian coordinates between two GPS points.
% This method is only valable for small distances

if size(lat1_dms,2) > 1
    % Convert dms to degrees
    lat1_d = lat1_dms(1) + lat1_dms(2)/60 + lat1_dms(3)/3600;
    lon1_d = lon1_dms(1) + lon1_dms(2)/60 + lon1_dms(3)/3600;
    lat2_d = lat2_dms(1) + lat2_dms(2)/60 + lat2_dms(3)/3600;
    lon2_d = lon2_dms(1) + lon2_dms(2)/60 + lon2_dms(3)/3600;
else
    lat1_d = lat1_dms;
    lon1_d = lon1_dms;
    lat2_d = lat2_dms;
    lon2_d = lon2_dms;
end

% Convert to radians
lat1 = lat1_d*pi/180;
lon1 = lon1_d*pi/180;
lat2 = lat2_d*pi/180;
lon2 = lon2_d*pi/180;

radius = 6371e3; %Earth's radius

deltaLat = lat2-lat1;
deltaLon = lon2-lon1;

x = radius*deltaLon*cos((lat1+lat2)/2);
y = radius*deltaLat;
end

