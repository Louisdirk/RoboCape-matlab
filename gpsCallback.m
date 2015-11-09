function gpsCallback(src,msg)

    global car_lat
    global car_lon
    global gps_new_data

    if ~isnan(msg.Latitude) || ~isnan(msg.Longitude)
        car_lat = msg.Latitude;
        car_lon = msg.Longitude;
        gps_new_data = 1;
    end
end

