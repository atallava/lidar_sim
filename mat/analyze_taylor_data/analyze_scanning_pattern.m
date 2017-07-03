% load section data
relPathSection = 'some_packets';
load(relPathSection,'section');

relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog');
poseTLog = tLog;

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

load('../data/laser_intrinsics','pitchVec');

%%
nPackets = length(section.packetIds);
packetIdx = randsample(1:nPackets,1);
t = section.packetTimestamps(packetIdx);
pts = getSectionPtsAtTime(section,t);
rayOrigin = [0 0 0]; % since points are in laser frame

[rayYawVec,rayPitchVec] = deal(zeros(1,size(pts,1)));
for j = 1:size(pts,1)
    rayDirn = calcRayDirn(rayOrigin,pts(j,:));
    [rayYawVec(j),rayPitchVec(j),~] = cart2sph(rayDirn(1),rayDirn(2),rayDirn(3));
end

%%
minYaw = min(rayYawVec);
flag = (rayYawVec >= minYaw) & (rayYawVec <= minYaw + 1e-3);
yawStart = mean(rayYawVec(flag));
maxYaw = max(rayYawVec);
flag = (rayYawVec >= maxYaw - 1e-3) & (rayYawVec <= maxYaw);
yawEnd = mean(rayYawVec(flag));

% initial estimate
yawVec = linspace(yawStart,yawEnd,12);

% map each point to closest yaw. then calculate mean for that bucket to
% update yawVec
nRays = length(rayYawVec);
nYaws = length(yawVec);
rayYawVec = flipVecToRow(rayYawVec);
mat1 = repmat(rayYawVec,nYaws,1);
yawVec = flipVecToColumn(yawVec);
mat2 = repmat(yawVec,1,nRays);
dmat = abs(mat1-mat2);
[~,yawMembershipIds] = min(dmat,[],1);
for i = 1:nYaws
    flag = (yawMembershipIds == i);
    yawVec(i) = mean(rayYawVec(flag));
end

%%
figure; 
marker = 'x';
markerSizeData = 30;
scatter(rayYawVec,rayPitchVec,'marker',marker,'sizeData',markerSizeData);

% pitch bars
xL = xlim; xL = flipVecToColumn(xL);
pitchVec = flipVecToRow(pitchVec);
pitchXMat = repmat(xL,1,length(pitchVec));
pitchYMat = [pitchVec; pitchVec];
hold on;
plot(pitchXMat,pitchYMat,'r');

% yaw bars
yL = ylim; yL = flipVecToColumn(yL);
yawVec = flipVecToRow(yawVec);
yawXMat = [yawVec; yawVec];
yawYMat = repmat(yL,1,length(yawVec));
hold on;
plot(yawXMat,yawYMat,'r');

xlabel('yaw (rad)'); ylabel('pitch (rad)');
titleStr = sprintf('packet idx: %d',packetIdx);
title(titleStr);


