% are the scan poses as expected along the path? how do the scans look in
% the world/ laser frame?

% load poses
sectionId = 4;
scansVersion = '300118';
dataSource = 'real';
sourceVersion = '';

relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, dataSource, sourceVersion);
[tPoses,TCell] = loadScanPoses(relPathScanPoses);

%% viz
hfig = figure(); hold on;
axes3Length = 10;
nPoses = length(TCell);
posns = zeros(nPoses,3);
for i = 1:nPoses
    T_laser = TCell{i};
    drawAxes3(hfig, T_laser, axes3Length);
    posns(i,:) = T_laser(1:3,4);
end

plot3(posns(:,1), posns(:,2), posns(:,3), '-');

text(posns(1,1), posns(1,2), posns(1,3), ...
    sprintf('t start: %d', tPoses(1)));
text(posns(end,1), posns(end,2), posns(end,3), ...
    sprintf('t end: %d', tPoses(end)));

axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

%% load some scans
scanIdsToViz = [0 100 300];
nScansToViz = length(scanIdsToViz);
scans_world = cell(1, nScansToViz);
scans_laser = cell(1, nScansToViz);

for i = 1:nScansToViz
    scanId = scanIdsToViz(i);
    relPathScan = genRelPathScan(sectionId,scansVersion,dataSource,sourceVersion,'world',scanId);
    scans_world{i} = loadPts(relPathScan);
    
    relPathScan = genRelPathScan(sectionId,scansVersion,dataSource,sourceVersion,'laser',scanId);
    scans_laser{i} = loadPts(relPathScan);
end

%% viz scans in world frame
hfig = figure(); hold on;
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

for i = 2%1:nScansToViz
    scanId = scanIdsToViz(i);
    scan_world = scans_world{i};
    scatter3(scan_world(:,1), scan_world(:,2), scan_world(:,3), '.');
    drawAxes3(hfig, TCell{scanId+1}, axes3Length);
end

% draw the posns 
plot3(posns(:,1), posns(:,2), posns(:,3));

title(sprintf('scanId: %d', scanId));

%% viz scans in laser frame
hfig = figure(); hold on;
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

for i = 2%nScansToViz
    scanId = scanIdsToViz(i);
    
    % recompute
    scan_world = scans_world{i};
    T_laser = TCell{scanId+1};
    scan_laser_ref = applyTransf(scan_world, inv(T_laser));
    scatter3(scan_laser_ref(:,1), scan_laser_ref(:,2), scan_laser_ref(:,3), 'b.');

    % from log
    scan_laser = scans_laser{i};
    scatter3(scan_laser(:,1), scan_laser(:,2), scan_laser(:,3), 'r.');
    
    % identity transform
    drawAxes3(hfig, eye(4,4), axes3Length);
end

title(sprintf('scanId: %d', scanId));










