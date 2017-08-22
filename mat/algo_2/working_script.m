relPathPoseLog = '../data/pose_log.mat';
load(relPathPoseLog,'poseLog','tLog');

%% get imu path
% section 4
% tExtents = [1403045911 1403046027];

tExtents = [1403045920 1403046033];

t = tExtents(1);
dt = 1;
posns = [];
while t < tExtents(2)
    imuPose = getImuPoseAtTime(poseLog,tLog,t);
    T_imu_world = getImuTransfFromImuPose(imuPose);
    xy = T_imu_world(1:2,4);
    xy = flipVecToRow(xy);
    posns = [posns; xy];
    t = t+dt;
end
%%
ar = polyarea(posns(:,1),posns(:,2));
dxy = diff(posns,1);
ds = sqrt(sum(dxy.^2,2));
s = sum(ds);
avgSpeed = s/(tExtents(2)-tExtents(1));

%% 
plot(posns(:,1),posns(:,2),'-+');
axis equal;
box on;
xlabel('x (m)'); ylabel('y (m)');




