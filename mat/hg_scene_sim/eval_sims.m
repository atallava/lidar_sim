sectionId = 4;
queryType = 'slice';
tag = -1;

simTypes = {'hg','hg','mm'};
simVersions = {'080917','280817','280817'};

%%
nSimsToEval = length(simTypes);
dispRangeErrorFlag = 1;
for i = 1:nSimsToEval
    simType = simTypes{i};
    simVersion = simVersions{i};
    fprintf('sim type: %s, version: %s\n',simType,simVersion);
    dispHorizontalLine(20);
    
    relPathSimDetail = genRelPathSimDetailMat(sectionId,simType,simVersion,queryType,tag);
    load(relPathSimDetail,'simDetail');
    calcRangeError(simDetail,dispRangeErrorFlag);
    dispHorizontalLine(40);
end