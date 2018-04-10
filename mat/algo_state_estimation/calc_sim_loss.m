sectionId = 4;
scansVersion = '300118';

dataSourceReal = 'real';
sourceVersionReal = '';

dataSourceSim = 'hg_sim';
sourceVersionSim = '080917';

% dataSourceSim = 'mm_sim';
% sourceVersionSim = '010218';

paramSamplesVersion = '18220218';

%% load reports
relPathSamples = genRelPathParamSamples(paramSamplesVersion);
paramSamples = loadParamSamples(relPathSamples);
nSamples = size(paramSamples,1);

relPathReportsPerParam = cell(1,nSamples);
for i = 1:nSamples
    paramIdx = i-1; % 0 indexing
    relPathReportsPerParam{i} = genRelPathMatchReports(sectionId, scansVersion, dataSourceReal, sourceVersionReal, paramSamplesVersion, paramIdx);
end
reportStructReal = loadMatchReportsPerParam(relPathReportsPerParam);

for i = 1:nSamples
    paramIdx = i-1; % 0 indexing
    relPathReportsPerParam{i} = genRelPathMatchReports(sectionId, scansVersion, dataSourceSim, sourceVersionSim, paramSamplesVersion, paramIdx);
end
reportStructSim = loadMatchReportsPerParam(relPathReportsPerParam);

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, dataSourceReal, sourceVersionReal);
[tPoses,TCellReal] = loadScanPoses(relPathScanPoses);

posnsReal = getPosnsFromTfs(TCellReal);

%% sim loss
[posnErrPerParamReal, rotnErrPerParamReal, posnsEstPerParamReal] = calcPoseErrorPerParam(TCellReal, reportStructReal.dispArrayPerParam);
[posnErrPerParamSim, rotnErrPerParamSim, posnsEstPerParamSim] = calcPoseErrorPerParam(TCellReal, reportStructSim.dispArrayPerParam);

simLosses = (posnErrPerParamReal-posnErrPerParamSim).^2;
simRisk = mean(simLosses);

fprintf('sim: %s\n', dataSourceSim);
fprintf('sim risk: %.3f\n', simRisk);
[maxLoss, maxId] = max(simLosses);
[minLoss, minId] = min(simLosses);
fprintf('max sim loss: %.3f, param id: %d\n', maxLoss, maxId);
fprintf('min sim loss: %.3f, param id: %d\n', minLoss, minId);
fprintf('\n');

%% viz
% pick param
paramsIdx = 73;
posnsEstReal = posnsEstPerParamReal{paramsIdx};
posnsEstSim = posnsEstPerParamSim{paramsIdx};

hfig = figure();
lineWidth = 1.5;
plot3(posnsReal(:,1),posnsReal(:,2),posnsReal(:,3),'b-', ...
    'linewidth',lineWidth);
hold on; axis equal;
plot3(posnsEstReal(:,1),posnsEstReal(:,2),posnsEstReal(:,3),'g-', ...
    'linewidth',lineWidth);
plot3(posnsEstSim(:,1),posnsEstSim(:,2),posnsEstSim(:,3),'r-', ...
    'linewidth',lineWidth);
xlabel('x (m)'); ylabel('y (m)');
legend({'ground truth','est real', 'est sim'});
view(2);
box on; grid on;
title(sprintf('params idx: %d', paramsIdx));



