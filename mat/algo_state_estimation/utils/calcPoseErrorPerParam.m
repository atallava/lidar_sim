function [posnErrorPerParam, rotnErrorPerParam, posnsEstPerParam] = calcPoseErrorPerParam(TCellReal, dispArrayPerParam)
%CALCPOSEERRORPERPARAM
%
% [posnErrorPerParam, rotnErrorPerParam, posnsEstPerParam] = CALCPOSEERRORPERPARAM(TCellReal, dispArrayPerParam)
%
% TCellReal         -
% dispArrayPerParam -
%
% posnErrorPerParam -
% rotnErrorPerParam -
% posnsEstPerParam  -

nSamples = length(dispArrayPerParam);

posnsReal = getPosnsFromTfs(TCellReal);

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
end