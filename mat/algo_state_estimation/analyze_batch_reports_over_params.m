sectionId = 4;
scansVersion = '300118';
dataSource = 'real';
sourceVersion = '';
paramSamplesVersion = '22190218';

%% load reports
relPathSamples = genRelPathParamSamples(paramSamplesVersion);
paramSamples = loadParamSamples(relPathSamples);
nSamples = size(paramSamples,1);
[successesPerParam, durationsPerParam, inlierRatiosPerParam, fitnessesPerParam, dispArrayPerParam] = ...
    deal(cell(1,nSamples));

for i = 1:nSamples
    paramIdx = i-1;
    relPathReports = genRelPathMatchReports(sectionId, scansVersion, dataSource, sourceVersion, paramSamplesVersion, paramIdx);
    [successes,durations,inlierRatios,fitnesses,dispArray] = loadMatchReports(relPathReports);
    
    successesPerParam{i} = successes;
    durationsPerParam{i} = durations;
    inlierRatiosPerParam{i} = inlierRatios;
    fitnessesPerParam{i} = fitnesses;
    dispArrayPerParam{i} = dispArray;
end

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, dataSource, sourceVersion);
[tPoses,TCellReal] = loadScanPoses(relPathScanPoses);

nPoses = length(TCellReal);
posnsReal = zeros(nPoses,3);
for i = 1:nPoses
    TReal = TCellReal{i};
    posnsReal(i,:) = TReal(1:3,4);
end

%% esimated posns
TCellEstPerParam = cell(1,nSamples);
posnsEstPerParam = cell(1,nSamples);
for i = 1:nSamples
    TCellEstPerParam{i} = calcTCellEst(TCellReal{1},dispArrayPerParam{i});
    posnsEstPerParam{i} = getPosnsFromTfs(TCellEstPerParam{i});
end

%% errors
posnErrorPerParam = zeros(1,nSamples);
for i = 1:nSamples
    posnErrorPerParam(i) = calcPosnError(posnsReal,posnsEstPerParam{i});
end

% some stats
[minPosnError, minErrorId] = min(posnErrorPerParam);
[maxPosnError, maxErrorId] = max(posnErrorPerParam);
fprintf('min posn error: %.3f id: %d\n', minPosnError, minErrorId);
fprintf('max posn error: %.3f, id: %d\n', maxPosnError, maxErrorId);
fprintf('mean error: %.3f\n', mean(posnErrorPerParam));

%% viz
% pick param
paramsIdx = 75;
posnsEst = posnsEstPerParam{paramsIdx};

hfig = figure();
lineWidth = 1.5;
plot(posnsReal(:,1),posnsReal(:,2),'bo-', ...
    'linewidth',lineWidth);
hold on; axis equal;
plot(posnsEst(:,1),posnsEst(:,2),'rx-', ...
    'linewidth',lineWidth);
xlabel('x (m)'); ylabel('y (m)');
legend({'real','est'});



