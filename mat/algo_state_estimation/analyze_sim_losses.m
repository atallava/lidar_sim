% hastily written script for some results analysis
load('error_calc_2_res', 'sectionId', 'scansVersion', 'dataSources', 'sourceVersions', 'paramSamplesVersion', ...
    'reportStructPerSource', 'posnErrPerParamPerSource', 'rotnErrPerParamPerSource');
nDataSources = length(dataSources);

relPathSamples = genRelPathParamSamples(paramSamplesVersion);
paramSamples = loadParamSamples(relPathSamples);
nSamples = size(paramSamples,1);

%% perf stats
% good and bad params for each data source
fprintf('posn error stats: \n\n');
for i = 1:nDataSources
    fprintf('data source: %s, version: %s\n', dataSources{i}, sourceVersions{i});
    posnErrPerParam = posnErrPerParamPerSource{i};
    dispQuickVecStats(posnErrPerParam);
end

%% sim losses
nSims = nDataSources-1;
simLossesPerSim = cell(1, nSims);
simRiskPerSim = zeros(1, nSims);

% posn error
% errPerParamReal = posnErrPerParamPerSource{1};

% rotn error
errPerParamReal = rotnErrPerParamPerSource{1};

% durations
% durationsPerParam = reportStructPerSource{1}.durationsPerParam;
% errPerParamReal = zeros(1, nSamples);
% for j = 1:nSamples
%     errPerParamReal(j) = mean(durationsPerParam{j});
% end

fprintf('sim loss stats: \n\n');
for i = 1:nSims
    fprintf('sim: %s, version: %s\n', dataSources{i+1}, sourceVersions{i+1});
    
    % posn error
%     errPerParamSim = posnErrPerParamPerSource{i+1};
%     simLosses = abs(errPerParamReal-errPerParamSim);
    
    % rotn error
    errPerParamSim = rotnErrPerParamPerSource{i+1};
    simLosses = abs(angdiff(errPerParamReal,errPerParamSim));
    
    % durations
%     durationsPerParam = reportStructPerSource{i+1}.durationsPerParam;
%     errPerParamSim = zeros(1, nSamples);
%     for j = 1:nSamples
%         errPerParamSim(j) = mean(durationsPerParam{j});
%     end
%     simLosses = abs(errPerParamReal-errPerParamSim);

    simRisk = mean(simLosses);
    
    dispQuickVecStats(simLosses);
    
    simLossesPerSim{i} = simLosses;
    simRiskPerSim(i) = simRisk;
end

%% del sim losses
% here shamelessly using my knowledge that there are only 2 sims
fprintf('del sim losses stats: \n\n');
delSimLosses = abs(simLossesPerSim{1} - simLossesPerSim{2});
dispQuickVecStats(delSimLosses);

%% viz bar
hfig = figure();
bar(simLossesPerSim{1}, 'r');
hold on;
bar(simLossesPerSim{2}, 'b');
xlim([1 100]);

legend('baseline', 'our approach');

% posn error
% ylabel('application-level loss (m)');
% xlabel('application parameter sample');

% rotn error
ylabel('application-level loss (rad)');
xlabel('application parameter sample');

set(gca,'fontsize',13);





