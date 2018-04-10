% quick script for thesis. viz ground truth path, and estimated paths for
% real and sims. write to file for all params.

sectionId = 4;
scansVersion = '300118';

dataSources = {'real', 'hg_sim', 'mm_sim'};
sourceVersions = {'', '080917', '010218'};

paramSamplesVersion = '18220218';

%% load reports
clockLocal = tic();
relPathSamples = genRelPathParamSamples(paramSamplesVersion);
paramSamples = loadParamSamples(relPathSamples);
nSamples = size(paramSamples,1);

nDataSources = length(dataSources);
reportStructPerSource = cell(1, nDataSources);

for i = 1:nDataSources
    dataSource = dataSources{i};
    sourceVersion = sourceVersions{i};
    fprintf('loading reports for data source: %s, version: %s\n', dataSource, sourceVersion);
    relPathReportsPerParam = cell(1,nSamples);
    for j = 1:nSamples
        paramIdx = j-1; % 0 indexing
        relPathReportsPerParam{j} = genRelPathMatchReports(sectionId, scansVersion, dataSource, sourceVersion, ...
            paramSamplesVersion, paramIdx);
    end
    reportStruct = loadMatchReportsPerParam(relPathReportsPerParam);
    reportStructPerSource{i} = reportStruct;
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n', compTime);

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, 'real', '');
[tPoses,TCellReal] = loadScanPoses(relPathScanPoses);

posnsReal = getPosnsFromTfs(TCellReal);

%% calc errors
clockLocal = tic();
[posnErrPerParamPerSource, rotnErrPerParamPerSource, posnsEstPerParamPerSource] = ...
    deal(cell(1, nDataSources));
for i = 1:nDataSources
    fprintf('calculating errors for data source: %s, version: %s\n', dataSources{i}, sourceVersions{i});
    reportStruct = reportStructPerSource{i};
    [posnErrPerParam, rotnErrPerParam, posnsEstPerParam] = calcPoseErrorPerParam(TCellReal, reportStruct.dispArrayPerParam);

    posnErrPerParamPerSource{i} = posnErrPerParam;
    rotnErrPerParamPerSource{i} = rotnErrPerParam;
    posnsEstPerParamPerSource{i} = posnsEstPerParam;
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n', compTime);

%% write figs
clockLocal = tic();
relPathFigsDir = '../figs/algo_state_estimation/est_per_source';
mkdir(relPathFigsDir);
lineWidth = 1.5;

for paramIdx = 1:nSamples
    fprintf('writing fig for param idx: %d\n', paramIdx);

    % ground truth traj
    hfig = figure(); hold on;
    plot3(posnsReal(:,1),posnsReal(:,2),posnsReal(:,3),'b-', ...
        'linewidth',lineWidth);

    % estimated traj
    hPlotPerSource = gobjects(1, nDataSources);
    for sourceIdx = 1:nDataSources
        posnsEstPerParam = posnsEstPerParamPerSource{sourceIdx};
        posnsEst = posnsEstPerParam{paramIdx};
        hPlotPerSource(sourceIdx) =  plot3(posnsEst(:,1),posnsEst(:,2),posnsEst(:,3), '-', ...
            'linewidth',lineWidth);
    end
    legend(hPlotPerSource, replaceUnderscoreWithSpace(dataSources));
    
    % fig properties
    axis equal;
    xlabel('x (m)'); ylabel('y (m)');
    view(2);
    box on; grid on;
    title(sprintf('param idx: %d', paramIdx));
    set(hfig, 'visible', 'off');
    
    % save
    relPathPng = [relPathFigsDir '/' num2str(paramIdx) '.png'];
    saveas(hfig, relPathPng);
    close(hfig);
end
fprintf('Written figs to %s\n', relPathFigsDir);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n', compTime);
    
%% save the work in computing errors    
% todo: makeshift, use path discipline
save('error_calc_res', 'sectionId', 'scansVersion', 'dataSources', 'sourceVersions', 'paramSamplesVersion', ...
    'reportStructPerSource','posnErrPerParamPerSource', 'rotnErrPerParamPerSource', 'posnsEstPerParamPerSource');
