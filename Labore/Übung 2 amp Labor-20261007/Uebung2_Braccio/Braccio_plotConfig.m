%% Visualisierung Ergebnisse

close all

braccioRobot.DataFormat = 'column';
figure



%%
% --- Konfiguration ---
perspective = 1;
switch perspective
    case 1
        perspectiveName = 'isometric';
        cameraView = [137.5, 25];
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

config = [0 90 0 90 0]'*pi/180;

figure('Color', 'w'); 
% 1. Roboter einmalig in Startkonfiguration zeichnen
ax = show(braccioRobot, config, 'Frames', 'off');
hold on;
% Kameraansicht für das Achsen-Handle explizit setzen
view(ax, cameraView);
grid off;
axis equal;
axis([-0.1 0.3 -0.1 0.4 -0.01 0.5])
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
