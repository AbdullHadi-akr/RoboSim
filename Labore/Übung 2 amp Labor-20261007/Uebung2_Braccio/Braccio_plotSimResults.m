%% Visualisierung Ergebnisse
braccioRobot.DataFormat = 'column';

startTime = 10;
stopTime = inf;
timeIdx = (out.ConfigSim.Time>=startTime & out.ConfigSim.Time<=stopTime);
time = out.ConfigSim.Time(timeIdx);

% Gefilterte Konfigurationsdaten vorbereiten
configData = out.ConfigSim.Data(timeIdx, :);

q1_deg = squeeze(configData(:,1)*180/pi);
q2_deg = squeeze(configData(:,2)*180/pi);
q3_deg = squeeze(configData(:,3)*180/pi);
q4_deg = squeeze(configData(:,4)*180/pi);
q5_deg = squeeze(configData(:,5)*180/pi);

figure
ai(1) = subplot(6,1,1);
plot(time,q1_deg)
ylim([-180 180])
grid on
ylabel('q1 [deg]')

ai(2) = subplot(6,1,2);
plot(time,q2_deg)
ylim([-180 180])
grid on
ylabel('q2 [deg]')

ai(3) = subplot(6,1,3);
plot(time,q3_deg)
ylim([-180 180])
grid on
ylabel('q3 [deg]')

ai(4) = subplot(6,1,4);
plot(time,q4_deg)
ylim([-180 180])
grid on
ylabel('q4 [deg]')

ai(5) = subplot(6,1,5);
plot(time,q5_deg)
ylim([-180 180])
grid on
ylabel('q5 [deg]')

%%
% --- Konfiguration ---
createVid = 1;                % 1 = Video "robotSim.mp4" erstellen, 0 = Nur anzeigen
showPath = 1;
perspective = 1;
switch perspective
    case 1
        perspectiveName = 'isometric';
        cameraView = [137.5, 20];
    case 2
        perspectiveName = 'front';
        cameraView = [90, 0];
    case 3
        perspectiveName = 'rightSide';
        cameraView = [180, 0];
    case 4
        perspectiveName = 'leftSide';
        cameraView = [0, 0];
    case 5
        perspectiveName = 'top';
        cameraView = [0, 90];
end
vidName = ['robotSim_' perspectiveName '.mp4'];

% Daten extrahieren (gefiltert über timeIdx)
xTCP = squeeze(out.TCPpos.Data(timeIdx,1));
yTCP = squeeze(out.TCPpos.Data(timeIdx,2));
zTCP = squeeze(out.TCPpos.Data(timeIdx,3));

% CtrlMode vorbereiten (auf gefilterte Simulationszeit interpolieren)
if isa(out.CtrlMode, 'timeseries') || (isobject(out.CtrlMode) && isprop(out.CtrlMode, 'Time'))
    ctrlModeVec = interp1(out.CtrlMode.Time, squeeze(out.CtrlMode.Data), time, 'nearest');
else
    ctrlModeVec = squeeze(out.CtrlMode.Data(timeIdx));
end

figure('Color', 'w'); 
% 1. Roboter einmalig in Startkonfiguration des gefilterten Intervalls zeichnen
ax = show(braccioRobot, configData(1,:)', 'Frames', 'off');
hold on;

if perspective>1
    camproj(ax, 'orthographic');
    ax.Box = 'off';
    ax.YGrid = 'off';
    ax.ZLabel.Position = [-0.26, 0, 0.25]; 
    ax.ZLabel.HorizontalAlignment = 'center';
end


% Kameraansicht für das Achsen-Handle explizit setzen
view(ax, cameraView);
%grid on;
axis([-0.2 0.5 -0.3 0.3 0 0.52])
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

% Initiale Zeit und Modus im Titel anzeigen
currentMode = ctrlModeVec(1);
title(ax, sprintf('t = %.1f s (CtrlMode: %d)', time(1), currentMode));

% 2. Erste animierte Linie für den Pfad initialisieren
hTrail = animatedline('Color', getModeColor(currentMode), 'LineWidth', 1.5, 'Marker', '.', 'MarkerSize', 8);

% 3. Video-Writer initialisieren (falls gewünscht)
if createVid
    vidObj = VideoWriter(vidName, 'MPEG-4');
    dt_sim = mean(diff(time));
    step_size = 10;
    vidObj.FrameRate = 1 / (dt_sim * step_size); 
    open(vidObj);
end

% 4. Schleife zur Animation
numSteps = length(time);
for kk = 1:10:numSteps
    currConfig = configData(kk,:)';
    modeNow = ctrlModeVec(kk);
    
    % Prüfen, ob sich der CtrlMode geändert hat
    if modeNow ~= currentMode
        currentMode = modeNow;
        % Neue animierte Linie erstellen, damit Teilstücke bei Moduswechsel NICHT verbunden werden
        hTrail = animatedline('Color', getModeColor(currentMode), 'LineWidth', 1.5, 'Marker', '.', 'MarkerSize', 8);
    end
    
    show(braccioRobot, currConfig, 'Parent', ax, 'FastUpdate', true, 'PreservePlot', false, 'Frames', 'off');
    
    if showPath
        addpoints(hTrail, xTCP(kk), yTCP(kk), zTCP(kk));
    end
    
    % Titel IMMER aktualisieren (mit Fallback für unbekannte Modi)
    switch currentMode
        case 1
            modeStr = 'Manual';
        case 3
            modeStr = 'Teach-In Replay';
        case 4
            modeStr = 'Pick-and-Place';
        case 5
            modeStr = 'Cartesian Control';
        otherwise
            modeStr = sprintf('Mode %d', currentMode);
    end
    title(ax, sprintf('t = %.1f s (%s)', time(kk), modeStr));
    
    drawnow;
    if createVid
        frame = getframe(gcf);
        writeVideo(vidObj, frame);
    end
end

if createVid
    close(vidObj);
    disp('Video erfolgreich erstellt.');
end
hold off;

% --- Hilfsfunktion für die Farbzuweisung ---
function col = getModeColor(mode)
switch mode
    case 1
        col = 'r'; % Rot bei 1 (manual)
    case 3
        col = 'b'; % Blau bei 3 (tech-in replay)
    case 4
        col = 'c'; % Cyan bei 4 (Pick-n-Place)
    case 5
        col = 'g'; % Grün bei 5 (cartesian control)
    otherwise
        col = 'k'; % Fallback (Schwarz) für andere Werte
end
end