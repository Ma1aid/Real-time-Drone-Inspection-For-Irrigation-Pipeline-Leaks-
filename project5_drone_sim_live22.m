function res = project5_drone_sim_live(params)
%% Drone Thermal Scan Simulation — Fixed v15 (Return color added)

%% ===== Default Parameters =====
d.rows = 200;
d.cols = 300;
d.baseTemp = 25;
d.leakStrength = 18;
d.leakSigma = 4;
d.noiseSigma = 1.2;
d.wpSpacing = 15;
d.scanRadius = 8;
d.seed = 0;
d.showFigures = true;
d.verticalSecPerMeter = 2.5;
d.horizontalSecPerMeter = 1.5;
d.returnSpeedMultiplier = 1.0;
d.updFreq = 1;
d.preallocDetect = 1;
d.sampleFactor = 0.5;
d.doPause = false;

if nargin < 1, params = struct(); end
p = d;
fn = fieldnames(params);
for k=1:numel(fn), p.(fn{k}) = params.(fn{k}); end

%% ===== RNG =====
if isempty(p.seed)
    rng('shuffle');
elseif ischar(p.seed) || isstring(p.seed)
    if strcmpi(p.seed,'shuffle'), rng('shuffle'); else rng(str2double(p.seed)); end
else
    rng(p.seed);
end

%% ===== Environment =====
rows = p.rows; cols = p.cols;
pipeline = false(rows, cols);
for c = 30:20:cols-30
    r = round(rows/2 + 20*sin(c/20));
    r = max(2,min(rows-1,r));
    pipeline(r-1:r+1,c) = true;
end
pipeline = imdilate(pipeline, strel('disk',2));
[pr,pc] = find(pipeline);

%% ===== Leaks =====
oldLeakRows = [112, 93, 86, 116, 82];
oldLeakCols = [50, 70, 110, 170, 210];
numLeaks = numel(oldLeakRows);

leakMask = false(rows, cols);
for i=1:numLeaks
    leakMask(oldLeakRows(i), oldLeakCols(i)) = true;
end

[X,Y] = meshgrid(1:cols, 1:rows);
T = p.baseTemp + 0.005*X + p.noiseSigma*randn(rows,cols);
for i=1:numLeaks
    lr = oldLeakRows(i); lc = oldLeakCols(i);
    T = T + p.leakStrength * exp(-((X-lc).^2 + (Y-lr).^2)/(2*p.leakSigma^2));
end

%% ===== Waypoints =====
[CX,CY] = meshgrid(1:p.wpSpacing:cols,1:p.wpSpacing:rows);
waypoints = [CY(:), CX(:)];
basePoint = waypoints(1,:);

%% ===== Figure Setup =====
if p.showFigures
    figure('Name','Drone Thermal Scan Live v15','Position',[100 100 900 500]);
    imagesc(T); colormap hot; colorbar; axis image; hold on;
    plot(pc,pr,'c-','LineWidth',2);

    scatter(oldLeakCols, oldLeakRows, 80, 'o', 'k', 'LineWidth',1.2);
    detectedLeakPlot = scatter([], [], 80, '*', 'b', 'LineWidth',1.5);

    droneLine  = animatedline('Color','w','LineStyle','--','LineWidth',1.2);
    returnLine = animatedline('Color','g','LineStyle','--','LineWidth',1.6); % ← NEW

    dronePlot = plot(basePoint(2), basePoint(1), 'w.-','MarkerSize',10,'LineWidth',1.5);
    scatter(basePoint(2), basePoint(1), 80, 'g', 'filled');
else
    detectedLeakPlot=[]; droneLine=[]; dronePlot=[]; returnLine=[];
end

%% ===== Simulation State =====
coverage = false(rows,cols);
alerts = zeros(0,2);
dronePath = basePoint;

%% ===== Mission =====
missionWaypoints = [basePoint; waypoints(2:end,:); basePoint];
nMoves = size(missionWaypoints,1)-1;

for i = 1:nMoves
    segStart = missionWaypoints(i,:);
    segEnd   = missionWaypoints(i+1,:);
    dist = norm(segEnd - segStart);

    if abs(segEnd(1)-segStart(1)) > abs(segEnd(2)-segStart(2))
        moveTime = dist * p.verticalSecPerMeter;
    else
        moveTime = dist * p.horizontalSecPerMeter;
    end
    if i==nMoves, moveTime = moveTime * p.returnSpeedMultiplier; end

    [segPath, coverage, leakMask, segAlerts] = droneMoveFast_v15( ...
        segStart, segEnd, moveTime, X, Y, p.scanRadius, leakMask, coverage, ...
        dronePlot, droneLine, returnLine, detectedLeakPlot, p, i==nMoves);

    if size(segPath,1)>1
        dronePath = vertcat(dronePath, segPath(2:end,:));
    end
    alerts = [alerts; segAlerts];
end

%% ===== Results =====
res.T = T;
res.pipeline = pipeline;
res.leakMask = leakMask;
res.alerts = alerts;
res.coverage = coverage;
res.dronePath = dronePath;
res.returnPath = [waypoints(end,:); basePoint];

%% ===== Coverage Map Figure =====
if p.showFigures
    coveragePercent = 100 * nnz(coverage) / numel(coverage);

    figure('Name','Coverage Map','Color','w','Position',[150 150 600 400]);
    imagesc(coverage);
    colormap([0 0 0; 1 1 1]);   % 0 = black (unscanned), 1 = white (scanned)
    axis image;
    set(gca,'XColor','k','YColor','k','Box','on');
    title(sprintf('Coverage %.1f%%', coveragePercent), 'Color','k');
end
end


%% ===== Movement Routine =====
function [segPath, coverage, leakMask, alerts] = droneMoveFast_v15( ...
    startPt, endPt, moveTime, X, Y, scanRadius, leakMask, coverage, ...
    dronePlot, droneLine, returnLine, detectedLeakPlot, p, isReturn)

alerts = zeros(0,2);
if norm(endPt - startPt)<1e-9, segPath=startPt; return; end

[nRows,nCols] = size(X);
scanR2 = scanRadius^2;

targetFPS=30;
desiredSteps=max(4, ceil(moveTime*targetFPS));
nSteps=max(4, round(desiredSteps/p.sampleFactor));
nSteps=min(nSteps,10000);

segPath=zeros(nSteps,2); segIdx=0;
detBuf=zeros(p.preallocDetect,2); detCnt=0;

r=ceil(scanRadius);
[DX,DY]=meshgrid(-r:r,-r:r);
mask=(DX.^2+DY.^2)<=scanR2;
linearOffsets=DY(mask)+DX(mask)*nRows;

traj = startPt + (endPt-startPt).*linspace(0,1,nSteps).';

for k=1:nSteps
    pos=traj(k,:);
    rC=min(max(round(pos(1)),1+r),nRows-r);
    cC=min(max(round(pos(2)),1+r),nCols-r);

    idxs = rC + (cC-1)*nRows + linearOffsets;
    coverage(idxs)=true;

    localLeaks = leakMask(idxs);
    if any(localLeaks)
        leakIdxs = idxs(localLeaks);
        [rD,cD]=ind2sub([nRows,nCols],leakIdxs);
        leakMask(leakIdxs)=false;
        detBuf(1:numel(rD),:)=[rD(:),cD(:)];
        detCnt=numel(rD);
    end

    segIdx=segIdx+1; segPath(segIdx,:)=pos;

    if detCnt>0 && isgraphics(detectedLeakPlot)
        xd=get(detectedLeakPlot,'XData'); yd=get(detectedLeakPlot,'YData');
        set(detectedLeakPlot,'XData',[xd(:); detBuf(1:detCnt,2)], ...
                              'YData',[yd(:); detBuf(1:detCnt,1)]);
        alerts=[alerts; detBuf(1:detCnt,:)];
        detCnt=0;
    end

    if isReturn
        if isgraphics(returnLine), addpoints(returnLine,pos(2),pos(1)); end
    else
        if isgraphics(droneLine), addpoints(droneLine,pos(2),pos(1)); end
    end

    if isgraphics(dronePlot)
        set(dronePlot,'XData',pos(2),'YData',pos(1));
    end
    drawnow limitrate
end

segPath=segPath(1:segIdx,:);
end