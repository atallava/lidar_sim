function posns = getPosnsFromTfs(TCell)
%GETPOSNSFROMTFS
%
% posns = GETPOSNSFROMTFS(TCell)
%
% TCell -
%
% posns -

nPoses = length(TCell);
posns = zeros(nPoses,3);
for i = 1:nPoses
    posns(i,:) = TCell{i}(1:3,4);
end
end