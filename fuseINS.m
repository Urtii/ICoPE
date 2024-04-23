function filter = fuseINS(filter, sensorData, noise)

    if all(~isnan(sensorData.Accelerometer(1,:)))
        fuseaccel(filter,sensorData.Accelerometer(1,:), ...
            noise.AccelerometerNoise);
    end
    if all(~isnan(sensorData.Gyroscope(1,:)))
        fusegyro(filter, sensorData.Gyroscope(1,:), ...
            noise.GyroscopeNoise);
    end
    if all(~isnan(sensorData.Magnetometer(1,:)))
    fusemag(filter,sensorData.Magnetometer(1,:), ...
            noise.MagnetometerNoise);
    end
    if all(~isnan(sensorData.GPSPosition(1,:)))
        noise.GPSPositionNoise = ...
                            eye(3)*[sensorData.GPSVelocity(1,1); ...
                                    sensorData.GPSVelocity(1,2); ...
                                    sensorData.GPSVelocity(1,3)];
        fusegps(filter,sensorData.GPSPosition(1,:), ...
                noise.GPSPositionNoise);
    end

end

