% viz error stats vs packet, for different sims
% potential packets for visualization

%% sims to eval
sectionId = 4;
queryType = 'slice';
tag = -1;

simTypes = {'hg','mm'};
simVersions = {'080917','280817'};
nSimsToEval = length(simTypes);

%% per packet stats
[packetErrorsCell,packetPrecisionsCell,packetRecallsCell,packetF1sCell] = deal(cell(1,nSimsToEval));
for i = 1:nSimsToEval
    simType = simTypes{i};
    simVersion = simVersions{i};
    relPathSimDetail = genRelPathSimDetailMat(sectionId,simType,simVersion,queryType,tag);
    load(relPathSimDetail,'simDetail');

    [packetErrors,packetPrecisions,packetRecalls] = ...
        calcRangeErrorPerPacket(simDetail);
    
    packetF1s = zeros(size(packetRecalls));
    for j = 1:length(packetRecalls)
        packetF1s(j) = calcF1Score(packetPrecisions(j), packetRecalls(j));
    end
    
    packetErrorsCell{i} = packetErrors;
    packetPrecisionsCell{i} = packetPrecisions;
    packetRecallsCell{i} = packetRecalls;
    packetF1sCell{i} = packetF1s;
end

%% viz
legendCell = cell(1,nSimsToEval);
for i = 1:nSimsToEval
    legendCell{i} = sprintf('%s %s',simTypes{i},simVersions{i});
end

originIdsToPlot = 1:10:length(packetErrorsCell{i});

figure; 
hold on;
for i = 1:nSimsToEval
    vec = packetErrorsCell{i};
    plot(originIdsToPlot,vec(originIdsToPlot),'.-');
end
legend(legendCell);
xlabel('origin ids'); ylabel('packet errors mean');

figure; 
hold on;
for i = 1:nSimsToEval
    vec = packetF1sCell{i}; 
    plot(originIdsToPlot,vec(originIdsToPlot),'.-');
end
legend(legendCell);
xlabel('origin ids'); ylabel('f1 score');


