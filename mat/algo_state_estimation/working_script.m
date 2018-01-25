%% load 
relPathSection = 'some_packets';
load(relPathSection,'section');

relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'poseLog','tLog');
poseTLog = tLog;

relPathLaserCalibParams = '../data/laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

relPathLaserIntrinsics = '../data/laser_intrinsics';
load(relPathLaserIntrinsics,'pitchVec');

%% yaw swept by each packet
nPackets = length(section.packetIds);
yawStartVec = zeros(1,nPackets);
yawEndVec = zeros(1,nPackets);
yawRangeVec = zeros(1,nPackets);

rayOrigin = [0 0 0]; % since points are in laser frame

for i = 1:nPackets
    t = section.packetTimestamps(i);
    pts = getSectionPtsAtTime(section,t);
    nPts = size(pts,1);
    
    % get yaws
    rayYaws = zeros(1,nPts);
    for j = 1:nPts
        rayDirn = calcRayDirn(rayOrigin,pts(j,:));
        [rayYaws(j),~,~] = cart2sph(rayDirn(1),rayDirn(2),rayDirn(3));
    end
    
    % yawEnd < yawStart since lidar rotates cw in own frame
    minYaw = min(rayYaws);
    flag = (rayYaws >= minYaw) & (rayYaws <= minYaw + 1e-3);
    yawEnd = mean(rayYaws(flag));
    maxYaw = max(rayYaws);
    flag = (rayYaws >= maxYaw - 1e-3) & (rayYaws <= maxYaw);
    yawStart = mean(rayYaws(flag));
    
    yawStartVec(i) = yawStart;
    yawEndVec(i) = yawEnd;
    yawRangeVec(i) = angdiff(yawEnd,yawStart);
end

%% yaw between packets
yawMissedBetweenPackets = zeros(1,nPackets-1);
for i = 1:(nPackets-1)
    yawMissedBetweenPackets(i) = angdiff(yawStartVec(i+1),yawEndVec(i));
end

%%
delYawPacket = median(yawRangeVec) + median(yawMissedBetweenPackets);
delYawAgg = 2*pi;
nPacketsToAgg = ceil(delYawAgg/delYawPacket);


