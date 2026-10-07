function robot_viz_update(u, waypoints)
% Visualize differential-drive robot state and signals during simulation.

persistent fig axTop axCmd axState
persistent hTraj hWaypoints hBody hWheelL hWheelR hCaster hArrow
persistent hV hW hX hY hTh
persistent tHist vHist wHist xHist yHist thHist
persistent figPosPx

defaultFigPosPx = [120 80 1100 760];

if nargin < 2
    waypoints = [];
end

if numel(u) < 6
    return;
end

x = u(2); y = u(4);
v = u(1); w = u(3);
t = u(6);

% Model provides angular values in rad and rad/s.
th = u(5);
wDeg = rad2deg(w);
thDeg = rad2deg(th);

R = 0.125;          % Chassis radius [m] (diameter 0.25 m)
wheelL = 0.08;      % Wheel length [m]
wheelW = 0.03;      % Wheel width [m]
rightAxisColor = [0.85 0.45 0.1];

if isempty(fig) || ~isvalid(fig)
    if isempty(figPosPx)
        figPosPx = defaultFigPosPx;
    end
    fig = figure('Name','Mobile Robot Visualization','NumberTitle','off','Color','w', ...
        'Units','pixels','Position',figPosPx);
    tl = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');

    axTop = nexttile(tl,1);
    hold(axTop,'on'); grid(axTop,'on'); axis(axTop,'equal');
    xlim(axTop,[0 8]); ylim(axTop,[0 4]);
    title(axTop,'2D Robot Pose and Path');
    xlabel(axTop,'x [m]'); ylabel(axTop,'y [m]');

    hTraj = plot(axTop,nan,nan,'r-','LineWidth',1.8);
    hWaypoints = plot(axTop,nan,nan,'o','Color',[0 0.447 0.741], ...
        'MarkerSize',6,'LineWidth',1.2,'HandleVisibility','off');
    hBody = plot(axTop,nan,nan,'k-','LineWidth',1.8);
    hWheelL = patch(axTop,nan,nan,[0.6 0.6 0.6],'EdgeColor','k');
    hWheelR = patch(axTop,nan,nan,[0.6 0.6 0.6],'EdgeColor','k');
    hCaster = plot(axTop,nan,nan,'ko','MarkerSize',6,'MarkerFaceColor',[0.9 0.9 0.9]);
    hArrow = quiver(axTop,0,0,0,0,0,'k','LineWidth',1.8,'MaxHeadSize',2);

    axCmd = nexttile(tl,2);
    hold(axCmd,'on'); grid(axCmd,'on');
    title(axCmd,'Command Signals');
    xlabel(axCmd,'t [s]');

    yyaxis(axCmd,'left');
    hV = plot(axCmd,nan,nan,'Color',[0.1 0.5 0.1],'LineWidth',1.2,'DisplayName','v');
    ylabel(axCmd,'v [m/s]');
    ylim(axCmd,[-0.5 0.5]);

    yyaxis(axCmd,'right');
    hW = plot(axCmd,nan,nan,'Color',rightAxisColor,'LineWidth',1.2,'DisplayName','\omega');
    ylabel(axCmd,'\omega [deg/s]');
    ylim(axCmd,[-60 60]);
    legend(axCmd,'Location','northeast','Interpreter','tex');

    axState = nexttile(tl,3);
    hold(axState,'on'); grid(axState,'on');
    title(axState,'States');
    xlabel(axState,'t [s]');

    yyaxis(axState,'left');
    hX = plot(axState,nan,nan,'r-','LineWidth',1.2,'DisplayName','x');
    hY = plot(axState,nan,nan,'m-','LineWidth',1.2,'DisplayName','y');
    ylabel(axState,'x,y [m]');
    ylim(axState,[0 8]);

    yyaxis(axState,'right');
    hTh = plot(axState,nan,nan,'-','Color',rightAxisColor,'LineWidth',1.2,'DisplayName','\theta');
    ylabel(axState,'\theta [deg]','Interpreter','tex');
    ylim(axState,[-360 360]);
    legend(axState,'Location','northeast','Interpreter','tex');

    tHist = []; vHist = []; wHist = [];
    xHist = []; yHist = []; thHist = [];

    [wpX, wpY] = localGetPurePursuitWaypoints(waypoints);
    if ~isempty(wpX)
        set(hWaypoints,'XData',wpX,'YData',wpY,'Visible','on');
    else
        set(hWaypoints,'Visible','off');
    end

    localAdjustFigureForEqualTop(fig, axTop, axCmd);

end

% Reset all histories/plots at the beginning of each simulation run.
if ~isempty(tHist) && t < tHist(end)
    tHist = []; vHist = []; wHist = [];
    xHist = []; yHist = []; thHist = [];

    set(hTraj,'XData',nan,'YData',nan);
    set(hV,'XData',nan,'YData',nan);
    set(hW,'XData',nan,'YData',nan);
    set(hX,'XData',nan,'YData',nan);
    set(hY,'XData',nan,'YData',nan);
    set(hTh,'XData',nan,'YData',nan);

    [wpX, wpY] = localGetPurePursuitWaypoints(waypoints);
    if ~isempty(wpX)
        set(hWaypoints,'XData',wpX,'YData',wpY,'Visible','on');
    else
        set(hWaypoints,'Visible','off');
    end

    localAdjustFigureForEqualTop(fig, axTop, axCmd);

end

% Append history
if isempty(tHist) || t >= tHist(end)
    tHist(end+1,1) = t;
    vHist(end+1,1) = v;
    wHist(end+1,1) = wDeg;
    xHist(end+1,1) = x;
    yHist(end+1,1) = y;
    thHist(end+1,1) = thDeg;
end

% Body circle
ang = linspace(0,2*pi,80);
set(hBody,'XData',x + R*cos(ang),'YData',y + R*sin(ang));

% Wheels (left/right)
leftCenter = [x;y] + [cos(th+pi/2); sin(th+pi/2)] * (R*0.82);
rightCenter = [x;y] + [cos(th-pi/2); sin(th-pi/2)] * (R*0.82);
rot = [cos(th) -sin(th); sin(th) cos(th)];
rectPts = 0.5*[-wheelL -wheelW; wheelL -wheelW; wheelL wheelW; -wheelL wheelW]';
wl = rot*rectPts + leftCenter;
wr = rot*rectPts + rightCenter;
set(hWheelL,'XData',wl(1,:),'YData',wl(2,:));
set(hWheelR,'XData',wr(1,:),'YData',wr(2,:));

% Rear passive wheel
casterPos = [x;y] - [cos(th); sin(th)] * (R*0.9);
set(hCaster,'XData',casterPos(1),'YData',casterPos(2));

% Heading arrow
arrLen = 0.22;
set(hArrow,'XData',x,'YData',y,'UData',arrLen*cos(th),'VData',arrLen*sin(th));

% Paths
set(hTraj,'XData',xHist,'YData',yHist);

% Time plots
set(hV,'XData',tHist,'YData',vHist);
set(hW,'XData',tHist,'YData',wHist);
set(hX,'XData',tHist,'YData',xHist);
set(hY,'XData',tHist,'YData',yHist);
set(hTh,'XData',tHist,'YData',thHist);

xlim(axCmd,[0 120]);
xlim(axState,[0 120]);

if ~isempty(fig) && isvalid(fig)
    fig.Units = 'pixels';
    figPosPx = fig.Position;
end

drawnow limitrate nocallbacks;
end

function localAdjustFigureForEqualTop(fig, axTop, axRef)
if isempty(fig) || ~isvalid(fig) || isempty(axTop) || ~isvalid(axTop) || isempty(axRef) || ~isvalid(axRef)
    return;
end

fig.Units = 'pixels';
axTop.Units = 'normalized';
axRef.Units = 'normalized';
drawnow limitrate nocallbacks;

figPos = fig.Position;
refPos = axRef.Position;

if refPos(3) <= 0 || refPos(4) <= 0 || figPos(4) <= 0
    return;
end

xLim = xlim(axTop);
yLim = ylim(axTop);
dx = abs(diff(xLim));
dy = abs(diff(yLim));
if dx <= 0 || dy <= 0
    return;
end

targetAxesRatio = dx / dy;
targetFigWidth = round(targetAxesRatio * (refPos(4) / refPos(3)) * figPos(4));
targetFigWidth = max(420, targetFigWidth);

if abs(figPos(3) - targetFigWidth) > 2
    fig.Position = [figPos(1), figPos(2), targetFigWidth, figPos(4)];
end
end

function [wpX, wpY] = localGetPurePursuitWaypoints(waypointsIn)
wpX = [];
wpY = [];

if isnumeric(waypointsIn) && ~isempty(waypointsIn) && size(waypointsIn,2) >= 2
    wpX = waypointsIn(:,1);
    wpY = waypointsIn(:,2);
    return;
end

mdl = 'mobile_robot_purePursuit';
blk = [mdl '/Constant'];
if ~bdIsLoaded(mdl)
    return;
end

if isempty(find_system(mdl,'SearchDepth',1,'Type','block','Name','Constant'))
    return;
end

valExpr = get_param(blk,'Value');

if isnumeric(valExpr)
    wp = valExpr;
elseif ischar(valExpr) || isstring(valExpr)
    expr = strtrim(char(valExpr));
    wp = [];

    % Prefer model workspace so Constant values like "waypoints" resolve correctly.
    try
        ws = get_param(mdl,'ModelWorkspace');
        wp = ws.evalin(expr);
    catch
    end

    if isempty(wp)
        try
            wp = evalin('base',expr);
        catch
        end
    end

    if isempty(wp)
        wp = str2num(expr); %#ok<ST2NM>
    end
else
    return;
end

if isnumeric(wp) && size(wp,2) >= 2 && ~isempty(wp)
    wpX = wp(:,1);
    wpY = wp(:,2);
end
end
