% approx dist covered by vehicle between two time instances

relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'poseLog','tLog');

%% start and end times
% tStart = 1403045836.830185000; 
% tEnd = 1403045902.775886000; % sec 3
% tStart = 1403045911.411350000; 
% tEnd = 1403046027.954617000; % sec 4
% tStart = 1403046151.135538000; 
% tEnd = 1403046221.65702000; % sec 6
tStart = 1403045583.453744888;
tEnd = 1403045648.386085033;

idStart = indexOfNearestTime(tStart,tLog);
idEnd = indexOfNearestTime(tEnd,tLog);

%% get xys
nPoses = size(poseLog,1);
duration = tEnd-tStart;
skip = 10;
ids = idStart:skip:idEnd;
xy = zeros(length(ids),2);
for i = 1:length(ids)
    id = ids(i);
    pose = poseLog(id,:);
    T = getImuTransfFromImuPose(pose);
    xy(i,:) = T(1:2,4);
end

%% calc 
dxy = diff(xy);
ds2 = sum(dxy.^2,2);
ds = sqrt(ds2);
s = sum(ds);
avgSpeed = s/duration;
fprintf('total dist: %.2f m, avg speed: %.2f m/s\n',s,avgSpeed);