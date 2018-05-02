function outStruct = loadMatchReportsPerParam(relPathReportsPerParam)
%LOADMATCHREPORTSPERPARAM
%
% outStruct = LOADMATCHREPORTSPERPARAM(relPathReportsPerParam)
%
% relPathReportsPerParam -
%
% outStruct              - struct
% ('successesPerParam','durationsPerParam','inlierRatiosPerParam','fitnessesPerParam','dispArrayPerParam')

nSamples = length(relPathReportsPerParam);
[successesPerParam, durationsPerParam, inlierRatiosPerParam, fitnessesPerParam, dispArrayPerParam] = ...
    deal(cell(1,nSamples));

for i = 1:nSamples
    relPathReports = relPathReportsPerParam{i};

    [successes,durations,inlierRatios,fitnesses,dispArray] = loadMatchReports(relPathReports);
    
    successesPerParam{i} = successes;
    durationsPerParam{i} = durations;
    inlierRatiosPerParam{i} = inlierRatios;
    fitnessesPerParam{i} = fitnesses;
    dispArrayPerParam{i} = dispArray;
end

outStruct.successesPerParam = successesPerParam;
outStruct.durationsPerParam = durationsPerParam;
outStruct.inlierRatiosPerParam = inlierRatiosPerParam;
outStruct.fitnessesPerParam = fitnessesPerParam;
outStruct.dispArrayPerParam = dispArrayPerParam;
end