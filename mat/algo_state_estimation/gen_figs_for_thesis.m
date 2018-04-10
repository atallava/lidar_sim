% hastily written script for some results analysis
load('error_calc_res', 'sectionId', 'scansVersion', 'dataSources', 'sourceVersions', 'paramSamplesVersion', ...
    'reportStructPerSource', 'posnErrPerParamPerSource', 'rotnErrPerParamPerSource', 'posnsEstPerParamPerSource');
nDataSources = length(dataSources);

relPathSamples = genRelPathParamSamples(paramSamplesVersion);
paramSamples = loadParamSamples(relPathSamples);
nSamples = size(paramSamples,1);

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, 'real', '');
[tPoses,TCellReal] = loadScanPoses(relPathScanPoses);

posnsReal = getPosnsFromTfs(TCellReal);

%% viz
paramIdx = 87;

lineWidth = 3;
fontSize = 20;
% ground truth traj
hfig = figure(); hold on;
plot3(posnsReal(:,1),posnsReal(:,2),posnsReal(:,3), ...
    'linewidth',lineWidth);

% estimated traj
posnsEstPerParamPerSource = posnsEstPerParamPerSource([1 3 2]);
hPlotPerSource = gobjects(1, nDataSources);
for sourceIdx = 1:nDataSources
    posnsEstPerParam = posnsEstPerParamPerSource{sourceIdx};
    posnsEst = posnsEstPerParam{paramIdx};
    hPlotPerSource(sourceIdx) =  plot3(posnsEst(:,1),posnsEst(:,2),posnsEst(:,3), '--', ...
        'linewidth',lineWidth);
end
legend(hPlotPerSource, replaceUnderscoreWithSpace(dataSources));

% fig properties
axis equal; box on;
xlabel('x (m)'); ylabel('y (m)');
view(2);
set(gca, 'fontsize', fontSize);