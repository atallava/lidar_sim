function posns = getPosnsFromTfs(TCell)
%GETPOSNSFROMTFS
%
% posns = GETPOSNSFROMTFS(TCell)
%
% TCell - cell array. TCell{i} is [4,4] homogeneous transform.
%
% posns - [nPoses, 3] array. xyz of each pose.

nPoses = length(TCell);
posns = zeros(nPoses,3);
for i = 1:nPoses
    posns(i,:) = TCell{i}(1:3,4);
end
end