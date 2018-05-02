% for a single set of parameters
sectionId = 4;
scansVersion = '300118';
dataSource = 'real';
sourceVersion = '';

%% load report
relPathReports = 'batch_reports.txt';
[successVec,durationVec,inlierRatioVec,fitnessVec,dispArray]  = loadMatchReports(relPathReports);

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, dataSource, sourceVersion);
[tPoses,TCellReal] = loadScanPoses(relPathScanPoses);

%% real, estimated posns
nPoses = length(TCellReal);
posnsReal = zeros(nPoses,3);
for i = 1:nPoses
    TReal = TCellReal{i};
    posnsReal(i,:) = TReal(1:3,4);
end

nDisps = nPoses-1;
TCellDisp = cell(1,nDisps);
for i = 1:nDisps
    T = quat2tform(dispArray(i,4:7));
    T(1:3,4) = dispArray(i,1:3);
    TCellDisp{i} = T;
end

TCellEst = cell(1,nDisps+1);
posnsEst = zeros(nDisps+1,3);
TCellEst{1} = TCellReal{1};
posnsEst(1,:) = TCellEst{1}(1:3,4);
for i = 2:(nDisps+1)
    TCellEst{i} = TCellEst{i-1}*TCellDisp{i-1};
    TEst = TCellEst{i};
    posnsEst(i,:) = TEst(1:3,4);
end

%% viz
hfig = figure();
lineWidth = 1.5;
plot(posnsReal(:,1),posnsReal(:,2),'bo-', ...
    'linewidth',lineWidth);
hold on; axis equal;
plot(posnsEst(:,1),posnsEst(:,2),'rx-', ...
    'linewidth',lineWidth);
xlabel('x (m)'); ylabel('y (m)');
legend({'real','est'});