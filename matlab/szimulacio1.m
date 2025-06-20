mqClient = mqttclient("tcp://192.168.82.241", ClientID="myClient", Port=1883);
mqClient.Connected
topicToWrite = "eesTopic";


lla0 = [46.5382, 24.5623, 320];  


lastGPS = lla0;


s = drivingScenario('GeoReference',lla0);      
v = vehicle(s);                               

waypoints = [-11 -0.25 0;                      
             -1 -0.25 0;
             -0.6 -0.4 0;
             -0.6 -9.3 0];

speed = [1.5;0.5;0.5;1.5];                      
smoothTrajectory(v,waypoints,speed);          

figure
plot(waypoints(:,1),waypoints(:,2),'-o')
xlabel('X (m)')
ylabel('Y (m)')
title('Vehicle Position Waypoints')





mountingLocationIMU = [1 2 3];                                         
mountingAnglesIMU = [0 0 0];                                            
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');  
imu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');   



mountingLocationGPS = [1 2 3];                                        
mountingAnglesGPS = [50 40 30];                                       
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame'); 
gps = gpsSensor('ReferenceLocation',lla0,'ReferenceFrame','ENU');     




accel = [];  
gyro = [];   
lla = [];    
gpsVel = [];
kalman_angles = [];  
raw_angles_without_noise = []; 
raw_angles_with_noise = [];

simSamplesPerGPS = (1/s.SampleTime)/gps.SampleRate; 
idx = 0;




simTime = 0;

logData = [];
idx = 0;


dt = s.SampleTime;  

A = [1, dt; 0, 1];

C = [1, 0];

%Q = [1e-5, 0; 0, 1e-3];  %1
%Q = [0.1, 0; 0, 0.2];    %2
%Q = [0.01, 0; 0, 0.1];   %3
Q = [0.05, 0; 0, 0.05];  %4
  

%R = 1;     %1
%R = 0.001; %2
%R = 0.1;   %3
R = 5;     %4


xhat = [0; 0];  
P = eye(2)  

filtered_angle = 0;
filtered_angular_velocity = 0;



while advance(s)
    groundTruth = state(v);                                                             

                                                                                        
    posVeh = groundTruth.Position;                                                       
    orientVeh = quaternion(fliplr(groundTruth.Orientation), 'eulerd', 'ZYX', 'frame');   
    velVeh = groundTruth.Velocity;                                                          
    accVeh = groundTruth.Acceleration;                                                      
    angvelVeh = deg2rad(groundTruth.AngularVelocity);                                         
    
   
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = transformMotion( ...
        mountingLocationIMU,orientVeh2IMU, ...
        posVeh,orientVeh,velVeh,accVeh,angvelVeh);                             
    [accel(end+1,:), gyro(end+1,:)] = imu(accIMU,angvelIMU,orientIMU); 
    
    imuEul = eulerd(orientIMU, 'ZYX', 'frame'); 
    imu_yaw = deg2rad(imuEul(1));
    imu_pitch = deg2rad(imuEul(2));
    imu_roll = deg2rad(imuEul(3));

    measured_angle = imu_yaw %%ez a valos szog zaj nelkul

    raw_angles_without_noise(end+1) = measured_angle;

    
    measured_angle_with_noise = measured_angle + 0.05*randn(1)

    raw_angles_with_noise(end+1) = measured_angle_with_noise;

    
    current_gz = gyro(end,3);
    
    
    
    % x̂⁻ₖ = A * x̂ₖ₋₁
    xhat_minus = A * xhat;
    
    % P⁻ₖ = A * Pₖ₋₁ * Aᵀ + Q
    P_minus = A * P * A' + Q;
    
    
    % Kₖ = P⁻ₖ * Cᵀ * (C * P⁻ₖ * Cᵀ + R)⁻¹
    K = P_minus * C' / (C * P_minus * C' + R);
    
    % x̂ₖ = x̂⁻ₖ + Kₖ * (zₖ - C * x̂⁻ₖ)
    xhat = xhat_minus + K * (measured_angle_with_noise - C * xhat_minus);
    
    % Pₖ = (I - Kₖ * C) * P⁻ₖ
    P = (eye(size(K*C)) - K * C) * P_minus;
    
    
    filtered_angle = xhat(1);
    filtered_angular_velocity = xhat(2);
    kalman_angles(end+1) = filtered_angle;

    
  
    if (mod(idx, simSamplesPerGPS) == 0)
        [posGPS,orientGPS,velGPS,accGPS,angvelGPS] = transformMotion(...
            mountingLocationGPS, orientVeh2GPS,...
            posVeh,orientVeh,velVeh,accVeh,angvelVeh);
        [lla(end+1,:), gpsVel(end+1,:)] = gps(posGPS,velGPS);

        lastGPS = lla(end, :); % Frissítjük az utolsó ismert GPS pozíciót
    end

                                                                                                                                                                                                 
                                                                                                                                                     
    [currentLat, currentLon, currentAlt] = ned2geodetic(posGPS(1), posGPS(2), posGPS(3), lla0(1), lla0(2), lla0(3), referenceEllipsoid('wgs84'));
    
    
    dataPoint = struct();

    dataPoint.gpsPos = [currentLat, currentLon, currentAlt];  
    
    dataPoint.imu2Angles = [imu_yaw, imu_roll, imu_pitch];
    
    dataPoint.imuAngles = [imu_yaw, imu_roll, imu_pitch];

    jsonData = jsonencode(dataPoint);

    write(mqClient, topicToWrite, jsonData);
    

    
    fprintf('GPS pozíció: [Lat: %.6f, Lon: %.6f, Alt: %.6f]\n', dataPoint.gpsPos(1), dataPoint.gpsPos(2), dataPoint.gpsPos(3));
    fprintf('IMU szögek (rad): [Yaw: %.3f, Roll: %.3f, Pitch: %.3f]\n', dataPoint.imuAngles(1), dataPoint.imuAngles(2), dataPoint.imuAngles(3));
    fprintf('Idő: %.2f s, Minta: %d\n', simTime, idx);
    fprintf('--------------------------------------------\n');

    idx = idx + 1;
    simTime = simTime + s.SampleTime; 
    logData = [logData; idx];
end

fprintf('A szimuláció időtartama: %.2f másodperc\n', simTime);
fprintf('Összes generált adat: %d\n', length(logData));


abs_error = abs(raw_angles_without_noise - kalman_angles);


mean_error = mean(abs_error);
max_error = max(abs_error);


fprintf('Átlagos abszolút hiba: %.6f rad (%.2f fok)\n', mean_error, rad2deg(mean_error));
fprintf('Maximális abszolút hiba: %.6f rad (%.2f fok)\n', max_error, rad2deg(max_error));



figure
plot(accel(:,1), 'r')  % X tengely - piros
hold on
plot(accel(:,2), 'g')  % Y tengely - zöld
plot(accel(:,3), 'b')  % Z tengely - kék
hold off
ylabel('Gyorsulás (m/s^2)')
xlabel('Minta index')
title('IMU Gyorsulás tengelyenként')
legend('X', 'Y', 'Z')
grid on

figure
plot(gyro)
ylabel('rad/s')
title('Gyroscope')

figure
geoplot(lla(:,1),lla(:,2))
title('GPS Position')

figure
plot(gpsVel)
ylabel('m/s')
title('GPS Velocity')

figure
plot(1:length(raw_angles_with_noise), raw_angles_with_noise, 'b-', ...
     1:length(kalman_angles), kalman_angles, 'r-')

legend('Nyers szög ', 'Kalman szűrt szög', 'FontSize', 18)
title('Szög értékek összehasonlítása', 'FontSize', 20)
xlabel('Minta', 'FontSize', 18)
ylabel('Szög (rad)', 'FontSize', 18)

set(gca, 'FontSize', 12)  % Tengelyek (x és y) skálafeliratainak mérete



figure
plot(abs_error, 'm')
title('Abszolút hiba a nyers és Kalman szűrt szög között')
xlabel('Minta index')
ylabel('Abszolút hiba (rad)')
grid on

%jelenlegi adatkuldes 100Hz(10ms)
