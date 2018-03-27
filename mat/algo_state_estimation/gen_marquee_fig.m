% for iros front page
% currently with placeholder clouds and guess disps

sectionId = 4;
scansVersion = '300118';
scansFrame = 'laser';
scanKeyId = 10;
scanQueryId = 11;

%% real
% relPathScan = genRelPathScan(sectionId, scansVersion, 'real', '', scansFrame, scanKeyId);
relPathScan = 'scan_000.xyz';
scanKey = loadPts(relPathScan);

% relPathScan = genRelPathScan(sectionId, scansVersion, 'real', '', scansFrame, scanQueryId);
relPathScan = 'scan_001.xyz';
scanQuery = loadPts(relPathScan);

%% aligned
dispEst = [1.977 0.206 -0.006 0.9997353659398454 -0.021922316823108478 0.006289533418433174 0.002961734373848484] ...
    + [10 -10 0 0 0 0 0];
T = quat2tform(dispEst(4:7));
T(1:3,4) = dispEst(1:3);

aligned = applyTransf(scanQuery,T);

%% sim
simName = 'mm';
simVersion = '123456';
relPathScan = genRelPathScan(sectionId, scansVersion, simName, simVersion, scansFrame, scanKeyId);
scanKey = loadPts(relPathScan);

relPathScan = genRelPathScan(sectionId, scansVersion, simName, simVersion, scansFrame, scanQueryId);
scanQuery = loadPts(relPathScan);

%% aligned
dispEst = [1.977 0.206 -0.006 0.9997353659398454 -0.021922316823108478 0.006289533418433174 0.002961734373848484];
T = quat2tform(dispEst(4:7));
T(1:3,4) = dispEst(1:3);

aligned = applyTransf(scanQuery,T);

%% viz
hfig = figure(); hold on;
skipForViz = 20;
markerSize = 300;
keyMarkerColor = [0 0 1];
alignedMarkerColor = [1 0 0];

scatter3(scanKey(1:skipForViz:end,1), scanKey(1:skipForViz:end,2), scanKey(1:skipForViz:end,3), '.', ...
    'sizeData', markerSize, 'markerEdgeColor', keyMarkerColor);
scatter3(aligned(1:skipForViz:end,1), aligned(1:skipForViz:end,2), aligned(1:skipForViz:end,3), '.', ...
    'sizeData', markerSize, 'markerEdgeColor', alignedMarkerColor);

axis equal; 
xlim([-57.0849   45.6695]); ylim([-54.9038   26.1396]); zlim([-10    10])

box on; grid off;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

% view(5, 38);
view(2);

fontSize = 40;
set(gca, 'fontSize', fontSize);

%% axes for help
% this comes from view 
hfig = figure();
T =  eye(4,4);
drawAxes3(hfig,T,10);
view(5, 38);