function [dm] = gpsDistBetweenPoints(lat1_dms, lon1_dms, lat2_dms, lon2_dms)

lat1_d = lat1_dms(1)*60 + lat1_dms(2) + lat1_dms(3)/60;
lon1_d = lon1_dms(1)*60 + lon1_dms(2) + lon1_dms(3)/60;
lat2_d = lat2_dms(1)*60 + lat2_dms(2) + lat2_dms(3)/60;
lon2_d = lon2_dms(1)*60 + lon2_dms(2) + lon2_dms(3)/60;

lat1 = lat1_d*pi/180;
lon1 = lon1_d*pi/180;
lat2 = lat2_d*pi/180;
lon2 = lon2_d*pi/180;

radius = 6371;

deltaLat = lat2-lat1;
deltaLon = lon2-lon1;

x = deltaLon*cos((lat1+lat2)/2);
y = deltaLat;
dm = radius*sqrt(x*x + y*y);
end

