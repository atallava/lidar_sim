sectionId = 4;
scansVersion = '300118';

% dataSource = 'real';
% sourceVersion = '';

% dataSource = 'hg_sim';
% sourceVersion = '080917';

dataSource = 'mm_sim';
sourceVersion = '010218';

paramSamplesVersion = '18220218';

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

rotnErrorPerParam = zeros(1,nSamples);
for i = 1:nSamples
    rotnErrorPerParam(i) = calcRotnError(TCellReal, TCellEstPerParam{i});
end

% some stats
fprintf('data source: %s\n', dataSource);

[minPosnError, minErrorId] = min(posnErrorPerParam);
[maxPosnError, maxErrorId] = max(posnErrorPerParam);
fprintf('min posn error: %.3f id: %d\n', minPosnError, minErrorId);
fprintf('max posn error: %.3f, id: %d\n', maxPosnError, maxErrorId);
fprintf('mean error: %.3f\n', mean(posnErrorPerParam));

[minRotnError, minErrorId] = min(rotnErrorPerParam);
[maxRotnError, maxErrorId] = max(rotnErrorPerParam);
fprintf('min rotn error: %.3f id: %d\n', minRotnError, minErrorId);
fprintf('max rotn error: %.3f, id: %d\n', maxRotnError, maxErrorId);
fprintf('mean error: %.3f\n', mean(rotnErrorPerParam));
fprintf('\n');

%% viz
% pick param
paramsIdx = 4;
posnsEst = posnsEstPerParam{paramsIdx};

hfig = figure();
lineWidth = 1.5;
plot3(posnsReal(:,1),posnsReal(:,2),posnsReal(:,3),'b-', ...
    'linewidth',lineWidth);
hold on; axis equal;
plot3(posnsEst(:,1),posnsEst(:,2),posnsEst(:,3),'r-', ...
    'linewidth',lineWidth);
xlabel('x (m)'); ylabel('y (m)');
legend({'real','est'});
view(2);
box on; grid on;
title(sprintf('params idx: %d', paramsIdx));



