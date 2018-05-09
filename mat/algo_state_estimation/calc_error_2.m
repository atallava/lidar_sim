% quick script for thesis and iser draft
% using the norms on relative poses here instead of absolute

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
    [posnErrPerParam, rotnErrPerParam] = calcPoseError2PerParam(TCellReal, reportStruct.dispArrayPerParam);

    posnErrPerParamPerSource{i} = posnErrPerParam;
    rotnErrPerParamPerSource{i} = rotnErrPerParam;
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n', compTime);

%% save the work in computing errors    
% todo: makeshift, use path discipline
save('error_calc_2_res', 'sectionId', 'scansVersion', 'dataSources', 'sourceVersions', 'paramSamplesVersion', ...
    'reportStructPerSource','posnErrPerParamPerSource', 'rotnErrPerParamPerSource');
