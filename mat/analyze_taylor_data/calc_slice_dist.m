relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'poseLog','tLog');

%%
nPoses = size(poseLog,1);
% tStart = 1403045836.830185000; % sec 3
% tEnd = 1403045902.775886000; % sec 3
tStart = 1403045911.411350000; % sec 4
tEnd = 1403046027.954617000; % sec 4
idStart = indexOfNearestTime(tStart,tLog);
idEnd = indexOfNearestTime(tEnd,tLog);
skip = 10;
ids = idStart:skip:idEnd;
xy = zeros(length(ids),2);
for i = 1:length(ids)
    id = ids(i);
    pose = poseLog(id,:);
    T = getImuTransfFromImuPose(pose);
    xy(i,:) = T(1:2,4);
end

%%
dxy = diff(xy);
ds2 = sum(dxy.^2,2);
ds = sqrt(ds2);
s = sum(ds);