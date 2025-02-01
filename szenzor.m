% MQTT kapcsolat létrehozása
mqClient = mqttclient("tcp://192.168.2.241", ClientID="myClient", Port=1883);
mqClient.Connected
topicToWrite = "eesTopic";

% IMU és GNSS paraméterek
imuFs = 1; % IMU mintavételi frekvencia [Hz]
gnssFs = 1;  % GNSS mintavételi frekvencia [Hz]
simTime = 10; % Szimulációs idő [s]
runtime = 0;  % Szimulációs időmérő változó

% Zajszintek
posNoise = 3; % GPS pozíció zaj szórása (m)
imuAccelNoise = 0.002; % IMU gyorsulásmérő zaj szórása (m/s^2)
imuGyroNoise = 0.001;  % IMU giroszkóp zaj szórása (rad/s)

% Idővektorok
t_imu = 0:1/imuFs:simTime; % IMU idővektor
t_gnss = 0:1/gnssFs:simTime; % GNSS idővektor

% Igazi pozíció (egyenes vonalú mozgás NED koordinátákban)
truePos = [0.1*t_imu', zeros(length(t_imu), 1), zeros(length(t_imu), 1)]; % [x,y,z]

% IMU adatok zajszimulációval
accelMeas = imuAccelNoise*randn(length(t_imu), 3); % Gyorsulásmérő mérés zajjal
gyroMeas = imuGyroNoise*randn(length(t_imu), 3); % Giroszkóp mérés zajjal

% GNSS pozíció zajszimulációval
gpsPos = truePos + posNoise*randn(size(truePos)); % GPS pozíció zajjal

% Adatok tárolása
simulationData = []; % Adatok tárolására szolgáló üres tömb

% Vizualizáció beállítása
figure;
hold on;

% GPS mérés
gpsPlot = plot3(gpsPos(:,1), gpsPos(:,2), gpsPos(:,3), 'ro', 'MarkerSize', 6, 'DisplayName', 'GPS mérés');

% IMU gyorsulásmérő adatai
imuAccelPlotX = plot3([], [], [], 'b-', 'DisplayName', 'IMU Gyorsulás X');
imuAccelPlotY = plot3([], [], [], 'g-', 'DisplayName', 'IMU Gyorsulás Y');
imuAccelPlotZ = plot3([], [], [], 'm-', 'DisplayName', 'IMU Gyorsulás Z');

% IMU giroszkóp adatai
imuGyroPlotX = plot3([], [], [], 'c-', 'DisplayName', 'IMU Giroszkóp X');
imuGyroPlotY = plot3([], [], [], 'y-', 'DisplayName', 'IMU Giroszkóp Y');
imuGyroPlotZ = plot3([], [], [], 'k-', 'DisplayName', 'IMU Giroszkóp Z');

% Grafikon beállítása
xlabel('Idő (s)');
ylabel('Érték');
zlabel('Tengely (m/s^2 vagy rad/s)');
grid on;
legend;
title('GPS és IMU mérések');
axis equal;

% Ciklus, amely folyamatosan frissíti az adatokat és küldi az MQTT üzenetet
while runtime < simTime
    runtime = runtime + 1/imuFs;  % Frissíti az időt
    currentTime = runtime;  % Aktuális idő
    
    % Adatpont frissítése (csak szenzoradatokkal)
    dataPoint = struct();
    dataPoint.gpsPos = gpsPos(round(currentTime * gnssFs) + 1, :); % GPS pozíció
    dataPoint.accelMeas = accelMeas(round(currentTime * imuFs) + 1, :); % Gyorsulásmérő
    dataPoint.gyroMeas = gyroMeas(round(currentTime * imuFs) + 1, :); % Giroszkóp
    
    % Adat hozzáadása a struktúrához
    simulationData = [simulationData; dataPoint];
    
    % Konzolra írás
    fprintf('IMU Idő: %.2f s\n', currentTime);
    fprintf('  Gyorsulás (m/s^2): [X: %.3f, Y: %.3f, Z: %.3f]\n', ...
        dataPoint.accelMeas(1), dataPoint.accelMeas(2), dataPoint.accelMeas(3));
    fprintf('  Giroszkóp (rad/s): [X: %.3f, Y: %.3f, Z: %.3f]\n', ...
        dataPoint.gyroMeas(1), dataPoint.gyroMeas(2), dataPoint.gyroMeas(3));
    fprintf('GPS Idő: %.2f s\n', currentTime);
    fprintf('  GPS Pozíció (m): [X: %.3f, Y: %.3f, Z: %.3f]\n', ...
        dataPoint.gpsPos(1), dataPoint.gpsPos(2), dataPoint.gpsPos(3));
    
    % JSON formátumba konvertálás (csak szenzoradatokkal)
    jsonData = jsonencode(dataPoint);
    
    % MQTT üzenet küldése
    write(mqClient, topicToWrite, jsonData);
    
    % Frissítés a vizualizációban
    set(gpsPlot, 'XData', gpsPos(1:round(currentTime*gnssFs)+1, 1), ...
        'YData', gpsPos(1:round(currentTime*gnssFs)+1, 2), ...
        'ZData', gpsPos(1:round(currentTime*gnssFs)+1, 3));
    
    set(imuAccelPlotX, 'XData', t_imu(1:round(currentTime*imuFs)), ...
        'YData', accelMeas(1:round(currentTime*imuFs), 1), 'ZData', accelMeas(1:round(currentTime*imuFs), 1));
    set(imuAccelPlotY, 'XData', t_imu(1:round(currentTime*imuFs)), ...
        'YData', accelMeas(1:round(currentTime*imuFs), 2), 'ZData', accelMeas(1:round(currentTime*imuFs), 2));
    set(imuAccelPlotZ, 'XData', t_imu(1:round(currentTime*imuFs)), ...
        'YData', accelMeas(1:round(currentTime*imuFs), 3), 'ZData', accelMeas(1:round(currentTime*imuFs), 3));
    
    set(imuGyroPlotX, 'XData', t_imu(1:round(currentTime*imuFs)), ...
        'YData', gyroMeas(1:round(currentTime*imuFs), 1), 'ZData', gyroMeas(1:round(currentTime*imuFs), 1));
    set(imuGyroPlotY, 'XData', t_imu(1:round(currentTime*imuFs)), ...
        'YData', gyroMeas(1:round(currentTime*imuFs), 2), 'ZData', gyroMeas(1:round(currentTime*imuFs), 2));
    set(imuGyroPlotZ, 'XData', t_imu(1:round(currentTime*imuFs)), ...
        'YData', gyroMeas(1:round(currentTime*imuFs), 3), 'ZData', gyroMeas(1:round(currentTime*imuFs), 3));
    
    % Frissítés
    drawnow;
    
    % Kis késleltetés (1 másodperc), hogy ne fusson túl gyorsan
    pause(1);
end
