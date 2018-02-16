% given two consecutive scans and the displacement between them, how do
% they look if overlaid?
sectionId = 4;
scansVersion = '300118';
dataSource = 'real';
sourceVersion = '';

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId,scansVersion,dataSource,sourceVersion);
[tScans, TCell] = loadScanPoses(relPathScanPoses);
nScans = length(tScans);

%% load scans
scanIdKey = randsample(nScans-1,1);
scanIdCloud = scanIdKey + 1;
scanIds = [scanIdKey scanIdCloud];

relPathScan = genRelPathScan(sectionId,scansVersion,dataSource,sourceVersion,'laser',scanIdKey);
scanKey = loadPts(relPathScan);

relPathScan = genRelPathScan(sectionId,scansVersion,dataSource,sourceVersion,'laser',scanIdCloud);
scanCloud = loadPts(relPathScan);

%% disp
T_key = TCell{scanIdKey};
T_cloud = TCell{scanIdCloud};
T_cloud_key = T_key\T_cloud;

% cloud pts in key frame
scanCloud_key = applyTransf(scanCloud, T_cloud_key);

%% viz
skipForViz = 20;

hfig = figure();
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

subplot(1,2,1); hold on;
scatter3(scanKey(1:skipForViz:end,1), scanKey(1:skipForViz:end,2), scanKey(1:skipForViz:end,3), '.');
scatter3(scanCloud(1:skipForViz:end,1), scanCloud(1:skipForViz:end,2), scanCloud(1:skipForViz:end,3), '.');
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('key','cloud')

subplot(1,2,2); hold on;
scatter3(scanKey(1:skipForViz:end,1), scanKey(1:skipForViz:end,2), scanKey(1:skipForViz:end,3), '.');
scatter3(scanCloud_key(1:skipForViz:end,1), scanCloud_key(1:skipForViz:end,2), scanCloud_key(1:skipForViz:end,3), '.');
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('key','cloud in key')

suptitle(sprintf('key id: %d, cloud id: %d', scanIdKey, scanIdCloud));
