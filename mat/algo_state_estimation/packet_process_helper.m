%% load 
relPathSection = 'some_packets';
% relPathSection = 'some_packets_section_04_subsampled';
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

%% specify agg yaw range
delYawPacket = median(yawRangeVec) + median(yawMissedBetweenPackets);
delYawAgg = 2*pi; % you pick this one
nPacketsToAgg = ceil(delYawAgg/delYawPacket);
fprintf('delYawPacket: %.2f\n',delYawPacket);
fprintf('delYawAgg: %.2f\n',delYawAgg);
fprintf('nPacketsToAgg: %.2f\n',nPacketsToAgg);

%% calc params
nNodes = 0; % calling number of (scan, pose) pairs after aggregation as nodes
aggDurationVec = [];
xyStartArray = [];
xyEndArray = [];
delXyVec = [];

aggIdxStart = 1;
while aggIdxStart < nPackets
    aggIdxEnd = aggIdxStart + nPacketsToAgg;
    if aggIdxEnd > nPackets
        break;
    end
    nNodes = nNodes+1;
    aggTStart = section.packetTimestamps(aggIdxStart);
    aggTEnd = section.packetTimestamps(aggIdxEnd);
    aggDurationVec(nNodes) = aggTEnd-aggTStart;
    
    imuPoseStart = getPoseAtTime(poseLog,tLog,aggTStart);
    laserTStart = getLaserTransfFromImuPose(imuPoseStart, ...
        laserCalibParams.extrinsics.TLaserImu);
    laserXyStart = laserTStart(1:2,4);
    xyStartArray(nNodes,:) = laserXyStart;
    
    imuPoseEnd = getPoseAtTime(poseLog,tLog,aggTEnd);
    laserTEnd = getLaserTransfFromImuPose(imuPoseEnd, ...
        laserCalibParams.extrinsics.TLaserImu);
    laserXyEnd = laserTEnd(1:2,4);
    xyEndArray(nNodes,:) = laserXyEnd;
    
    delXy = norm(laserXyEnd-laserXyStart);
    delXyVec(nNodes) = delXy;
    
    aggIdxStart = aggIdxEnd+1;
end

delXyStartArray = diff(xyStartArray);
delXyStartVec = zeros(1,nNodes-1);
for i = 1:(nNodes-1)
    delXyStartVec(i) = norm(delXyStartArray(i));
end

meanDelXy = mean(delXyVec);
meanAggDuration = mean(aggDurationVec);
meanDelXyStart = mean(delXyStartVec);

fprintf('delYawAgg: %.2f\n',delYawAgg);
fprintf('nPacketsToAgg: %.2f\n',nPacketsToAgg);
fprintf('nNodes: %.2f\n',nNodes);
fprintf('meanAggDuration: %.2f\n',meanAggDuration);
fprintf('meanDelXy: %.2f\n',meanDelXy);
fprintf('meanDelXyStart: %.2f\n',meanDelXyStart);
     
%% plot
hfig = figure;
plot(xyStartArray(:,1),xyStartArray(:,2),'rx');
hold on;
plot(xyEndArray(:,1),xyEndArray(:,2),'bo');
axis equal;
xlabel('x (m)'); ylabel('y (m)');



