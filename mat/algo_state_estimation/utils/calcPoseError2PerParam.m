function [posnErrorPerParam, rotnErrorPerParam] = calcPoseError2PerParam(TCellReal, dispArrayPerParam)
%CALCPOSEERROR2PERPARAM Like calcPoseErrorPerParam, except i use relative
% poses
%
% [posnErrorPerParam, rotnErrorPerParam] = CALCPOSEERROR2PERPARAM(TCellReal, dispArrayPerParam)
%
% TCellReal         -
% dispArrayPerParam -
%
% posnErrorPerParam -
% rotnErrorPerParam -

nSamples = length(dispArrayPerParam);

%% real displacements
TDispCellReal = calcTDispCellFromTCell(TCellReal);

%% estimated displacements
TDispCellEstPerParam = cell(1, nSamples);
for i = 1:nSamples
    TDispCellEstPerParam{i} = calcTsFromQuaternionPoses(dispArrayPerParam{i});
end

%% errors
posnErrorPerParam = zeros(1, nSamples);
rotnErrorPerParam = zeros(1, nSamples);
for i = 1:nSamples
    [posnErrorPerParam(i), rotnErrorPerParam(i)] = calcPosesError(TDispCellReal, TDispCellEstPerParam{i});
end
end