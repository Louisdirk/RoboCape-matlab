format long

lat1_dms = [46 31 12.37];
lon1_dms = [6 33 58.79];
lat2_dms = [46 31 12.15];
lon2_dms = [6 33 57.55];

lat1 = lat1_dms(1)*60 + lat1_dms(2) + lat1_dms(3)/60;
lon1 = lon1_dms(1)*60 + lon1_dms(2) + lon1_dms(3)/60;
lat2 = lat2_dms(1)*60 + lat2_dms(2) + lat2_dms(3)/60;
lon2 = lon2_dms(1)*60 + lon2_dms(2) + lon2_dms(3)/60;

radius=6371;

deltaLat=lat2-lat1;
deltaLon=lon2-lon1;

x=deltaLon*cos((lat1+lat2)/2);
y=deltaLat;
d2m=radius*sqrt(x*x + y*y);
