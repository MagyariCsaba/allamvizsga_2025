mqClient = mqttclient("tcp://192.168.18.241", ClientID="myClient", Port=1883);
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




%2)szenzorok definialasa
%a) IMU
mountingLocationIMU = [1 2 3];                                         
mountingAnglesIMU = [0 0 0];                                            
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame'); 
imu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');   


%b) GPS
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

simSamplesPerGPS = (1/s.SampleTime)/gps.SampleRate; %GPS mintavételezési arány kiszámítása
idx = 0;



simTime = 0; % Idő számláló

logData = [];
idx = 0;

dt = s.SampleTime;  % mintavételezési idő

A = [1, dt; 0, 1];

C = [1, 0];

Q = [1e-5, 0; 0, 1e-3];  %1
%Q = [0.1, 0; 0, 0.2];    %2
%Q = [0.01, 0; 0, 0.1];   %3
%Q = [0.05, 0; 0, 0.05];  %4
  
R = 1;     %1
%R = 0.001; %2
%R = 0.1;   %3
%R = 5;     %4



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
    
    
    current_ax = accel(end,1);
    current_ay = accel(end,2);
    measured_angle = atan2(current_ay, current_ax) 

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

                                                                                                                                                       
    dataPoint = struct();
    dataPoint.imuAccel = accel(end, :);                                                                                                               
    dataPoint.imuGyro = gyro(end, :);                                                                                                                 
                                                                                                                                                    
    [currentLat, currentLon, currentAlt] = ned2geodetic(posGPS(1), posGPS(2), posGPS(3), lla0(1), lla0(2), lla0(3), referenceEllipsoid('wgs84'));
    dataPoint.gpsPos = [currentLat, currentLon, currentAlt];                                                                                         
                                                                                                                                                                                                                                                                                                         %plusz imu konstans 0-kal
    dataPoint.imu2Accel = [-1, 0, 0];                                                                                                                     
    dataPoint.imu2Gyro = [0, 0, 0];                                                                                                                    

    jsonData = jsonencode(dataPoint);                                                                                                                      

    if mod(idx,10) == 0
        write(mqClient, topicToWrite, jsonData);
    end
    

    fprintf('IMU gyorsulás (m/s^2): [X: %.3f, Y: %.3f, Z: %.3f]\n', dataPoint.imuAccel(1), dataPoint.imuAccel(2), dataPoint.imuAccel(3));
    fprintf('IMU giroszkóp (rad/s): [X: %.3f, Y: %.3f, Z: %.3f]\n', dataPoint.imuGyro(1), dataPoint.imuGyro(2), dataPoint.imuGyro(3));
    fprintf('GPS pozíció (m): [X: %.6f, Y: %.6f, Z: %.6f]\n', dataPoint.gpsPos(1), dataPoint.gpsPos(2), dataPoint.gpsPos(3));
    fprintf('Szög (rad) - Nyers: %.3f, Kalman: %.3f\n', measured_angle, filtered_angle);
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

legend('Nyers szög (atan2)', 'Kalman szűrt szög', 'FontSize', 18)
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

%jelenlegi adatkuldes 100Hz(10ms)%mqtt inicializalas
