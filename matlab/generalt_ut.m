%mqtt inicializalas
mqClient = mqttclient("tcp://192.168.2.241", ClientID="myClient", Port=1883);
mqClient.Connected
topicToWrite = "eesTopic";

% Utolsó ismert GPS adat tárolása
lastGPS = [0, 0, 0]; % Kezdetben üres GPS adat


%1)szimulacios kornyezet
lla0 = [42 -71 50];                            % Kezdő georeferencia pont (szélesség, hosszúság, magasság)
s = drivingScenario('GeoReference',lla0);      %szimulációs környezet
v = vehicle(s);                                %Hozzáad egy járművet

waypoints = [-11 -0.25 0;                      %útvonal pontokat tartalmazó mátrix
             -1 -0.25 0;
             -0.6 -0.4 0;
             -0.6 -9.3 0];


speed = [1.5;0;0.5;1.5];                      %sebesseg matrix
smoothTrajectory(v,waypoints,speed);          % jármű mozgatása az útvonalon

figure
plot(waypoints(:,1),waypoints(:,2),'-o')
xlabel('X (m)')
ylabel('Y (m)')
title('Vehicle Position Waypoints')




%2)szenzorok definialasa
%a) IMU
mountingLocationIMU = [1 2 3];             %IMU érzékelő helyzetét adja meg a jármű koordinátarendszerében
mountingAnglesIMU = [0 0 0];               %IMU érzékelő dőlésszögeit (Euler-szögek) adja meg fokokban
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');  %Euler-szögek konvertálása kvaternióba
imu = imuSensor('SampleRate',1/s.SampleTime,'ReferenceFrame','ENU');   %IMU szenzor létrehozása


%b) GPS
mountingLocationGPS = [1 2 3];             %GPS érzékelő helyzete a jármű koordinátáihoz képest
mountingAnglesGPS = [50 40 30];            %GPS érzékelő dőlésszögei fokokban
orientVeh2GPS = quaternion(mountingAnglesGPS,'eulerd','ZYX','frame'); %Euler-szögek konvertálása kvaternióba
gps = gpsSensor('ReferenceLocation',lla0,'ReferenceFrame','ENU');     %GPS szenzor létrehozása


%c)Kerékfordulat-érzékelő
encoder = wheelEncoderAckermann('TrackWidth',v.Width,...
    'WheelBase',v.Wheelbase,'SampleRate',1/s.SampleTime);




%3)érzékelők adatainak gyűjtese, miközben a jármű a szimulációban mozog.
%tombok az adatoknak
accel = [];  %IMU gyorsulási adatok
gyro = [];   %IMU giroszkóp adatok (szögsebesség)
ticks = [];  %Kerékfordulat-érzékelő adatok
lla = [];    %GPS helyadatok (földrajzi koordináták)
gpsVel = []; %GPS sebességadatok

simSamplesPerGPS = (1/s.SampleTime)/gps.SampleRate; %GPS mintavételezési arány kiszámítása
idx = 0;

while advance(s)
    groundTruth = state(v);  %aktuális járműállapot

    %Jármű állapotának átalakítása
    posVeh = groundTruth.Position;
    orientVeh = quaternion(fliplr(groundTruth.Orientation), 'eulerd', 'ZYX', 'frame');
    velVeh = groundTruth.Velocity;
    accVeh = groundTruth.Acceleration;
    angvelVeh = deg2rad(groundTruth.AngularVelocity);
    
    %IMU érzékelő adatainak generálása
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = transformMotion( ...
        mountingLocationIMU,orientVeh2IMU, ...
        posVeh,orientVeh,velVeh,accVeh,angvelVeh);
    [accel(end+1,:), gyro(end+1,:)] = imu(accIMU,angvelIMU,orientIMU); 
    
    %Kerékfordulat-érzékelő adatainak generálása
    ticks(end+1,:) = encoder(velVeh, angvelVeh, orientVeh); 
    
    %GPS adatainak generálása
    if (mod(idx, simSamplesPerGPS) == 0)
        % Convert motion quantities from vehicle frame to GPS frame.
        [posGPS,orientGPS,velGPS,accGPS,angvelGPS] = transformMotion(...
            mountingLocationGPS, orientVeh2GPS,...
            posVeh,orientVeh,velVeh,accVeh,angvelVeh);
        [lla(end+1,:), gpsVel(end+1,:)] = gps(posGPS,velGPS);

        lastGPS = lla(end, :); % Frissítjük az utolsó ismert GPS pozíciót
    end

     % **Minden mintavételezéskor küldeni az adatokat MQTT-n**
    dataPoint = struct();
    dataPoint.imuAccel = accel(end, :);  % IMU gyorsulás (X,Y,Z)
    dataPoint.imuGyro = gyro(end, :);    % IMU giroszkóp (X,Y,Z)
    dataPoint.gpsPos = lastGPS;          % GPS pozíció (X,Y,Z) (mindig az utolsó ismert)
    %plusz imu konstans 0-kal
    dataPoint.imu2Accel = [0, 0, 0];  % Második IMU gyorsulás
    dataPoint.imu2Gyro = [0, 0, 0];   % Második IMU giroszkóp

    jsonData = jsonencode(dataPoint); % JSON formátumba alakítás
    write(mqClient, topicToWrite, jsonData); % MQTT üzenet küldése

    % Debug: kiíratás a konzolra
    fprintf('IMU gyorsulás (m/s^2): [X: %.3f, Y: %.3f, Z: %.3f]\n', dataPoint.imuAccel(1), dataPoint.imuAccel(2), dataPoint.imuAccel(3));
    fprintf('IMU giroszkóp (rad/s): [X: %.3f, Y: %.3f, Z: %.3f]\n', dataPoint.imuGyro(1), dataPoint.imuGyro(2), dataPoint.imuGyro(3));
    fprintf('GPS pozíció (m): [X: %.6f, Y: %.6f, Z: %.6f]\n', dataPoint.gpsPos(1), dataPoint.gpsPos(2), dataPoint.gpsPos(3));
    fprintf('--------------------------------------------\n');

    idx = idx + 1;
end




%4)vizualizacio

figure
plot(ticks)
ylabel('Wheel Ticks')
title('Wheel Encoder')

figure
plot(accel)
ylabel('m/s^2')
title('Accelerometer')

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
