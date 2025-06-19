mqClient = mqttclient("tcp://192.168.176.241", ClientID="myClient", Port=1883);
mqClient.Connected
topicToWrite = "eesTopic";


lla0 = [46.5382, 24.5623, 320];  

% Kezdő georeferencia pont (szélesség, hosszúság, magasság)
s = drivingScenario('GeoReference',lla0);      
v = vehicle(s);                                

waypoints = [-11 -0.25 0;                      
             -7 -0.25 0;
             -0.6 -0.4 0;
             -0.6 -5.3 0];

speed = [1.5;0.5;0.5;1.5];   
smoothTrajectory(v,waypoints,speed);         

%figure
%plot(waypoints(:,1),waypoints(:,2),'-o', 'LineWidth', 2, 'MarkerSize', 8)
%grid on
%axis equal
%xlabel('X (m) - East')
%ylabel('Y (m) - North')
%title('Vehicle Position Waypoints - Planned Yaw Changes')


mountingLocationIMU = [1 2 3];                                         
mountingAnglesIMU = [0 0 0];                                            
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');  
imu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');   

mountingLocationGPS = [1 2 3];                                        
mountingAnglesGPS = [50 40 30];                                       
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame'); 
gps = gpsSensor('ReferenceLocation',lla0,'ReferenceFrame','ENU');     

lla = [];    
simSamplesPerGPS = (1/s.SampleTime)/gps.SampleRate;
idx = 0;
simTime = 0; 

while advance(s)
    groundTruth = state(v);                                                             

    posVeh = groundTruth.Position;                                                       
    orientVeh = quaternion(fliplr(groundTruth.Orientation), 'eulerd', 'ZYX', 'frame');   
    velVeh = groundTruth.Velocity;                                                          
    accVeh = groundTruth.Acceleration;                                                      
    angvelVeh = deg2rad(groundTruth.AngularVelocity);                                         
    
    eul = eulerd(orientVeh, 'ZYX', 'frame');  % [yaw pitch roll] formátumban
    yaw_deg = eul(1);   % yaw fokban
    pitch_deg = eul(2); % pitch fokban  
    roll_deg = eul(3);  % roll fokban
  
    yaw_rad = deg2rad(yaw_deg);
    pitch_rad = deg2rad(pitch_deg);
    roll_rad = deg2rad(roll_deg);

    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = transformMotion( ...
        mountingLocationIMU,orientVeh2IMU, ...
        posVeh,orientVeh,velVeh,accVeh,angvelVeh);                             
    [accel, gyro] = imu(accIMU,angvelIMU,orientIMU); 
    
    imuEul = eulerd(orientIMU, 'ZYX', 'frame');  
    imu_yaw = deg2rad(imuEul(1));
    imu_pitch = deg2rad(imuEul(2));
    imu_roll = deg2rad(imuEul(3));
    
    if (mod(idx, simSamplesPerGPS) == 0)
        [posGPS,orientGPS,velGPS,accGPS,angvelGPS] = transformMotion(...
            mountingLocationGPS, orientVeh2GPS,...
            posVeh,orientVeh,velVeh,accVeh,angvelVeh);
        [lla(end+1,:), gpsVel] = gps(posGPS,velGPS);
    end

    
    
    [currentLat, currentLon, currentAlt] = ned2geodetic(posGPS(1), posGPS(2), posGPS(3), lla0(1), lla0(2), lla0(3), referenceEllipsoid('wgs84'));

    
    dataPoint = struct();

    dataPoint.gpsPos = [currentLat, currentLon, currentAlt];  
    
    dataPoint.imu2Angles = [imu_yaw, imu_roll, imu_pitch];
    
    dataPoint.imuAngles = [imu_yaw, imu_roll, imu_pitch];

    jsonData = jsonencode(dataPoint);

    write(mqClient, topicToWrite, jsonData);
    
    fprintf('GPS pozíció: [Lat: %.6f, Lon: %.6f, Alt: %.6f]\n', dataPoint.gpsPos(1), dataPoint.gpsPos(2), dataPoint.gpsPos(3));
    fprintf('Jármű yaw szög: %.1f° (%.3f rad)\n', yaw_deg, yaw_rad);
    fprintf('IMU szögek (rad): [Yaw: %.3f, Roll: %.3f, Pitch: %.3f]\n', dataPoint.imuAngles(1), dataPoint.imuAngles(2), dataPoint.imuAngles(3));
    fprintf('Idő: %.2f s, Minta: %d\n', simTime, idx);
    fprintf('--------------------------------------------\n');

    idx = idx + 1;
    simTime = simTime + s.SampleTime; 
end

fprintf('A szimuláció időtartama: %.2f másodperc\n', simTime);

%vizualizacio
%if ~isempty(lla)
  %  figure
  %  geoplot(lla(:,1),lla(:,2))
   % title('GPS Position')
%end
