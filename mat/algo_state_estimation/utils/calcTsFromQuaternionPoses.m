function TCell = calcTsFromQuaternionPoses(poseArray)
%CALCTSFROMQUATERNIONPOSES
%
% TCell = CALCTSFROMQUATERNIONPOSES(poseArray)
%
% poseArray - [nPoses, 7]. [1:3] is [xyz], [4:7] is quaternion
%
% TCell     -

nPoses = size(poseArray, 1);
TCell = cell(1, nPoses);
for i = 1:nPoses
    pose = poseArray(i, :);
    T = quat2tform(pose(4:7));
    T(1:3, 4) = pose(1:3);
    TCell{i} = T;
end
end