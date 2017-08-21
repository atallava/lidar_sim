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
minYawVec = zeros(1,nPackets);
tVec = zeros(1,nPackets);
for i = 1:nPackets
    t = section.packetTimestamps(i);
    pts = getSectionPtsAtTime(section,t);
    rayOrigin = [0 0 0]; % since points are in laser frame
    
    [rayYawVec,rayPitchVec] = deal(zeros(1,size(pts,1)));
    for j = 1:size(pts,1)
        rayDirn = calcRayDirn(rayOrigin,pts(j,:));
        [rayYawVec(j),rayPitchVec(j),~] = cart2sph(rayDirn(1),rayDirn(2),rayDirn(3));
    end
    minYaw = min(rayYawVec);
    
    minYawVec(i) = minYaw;
    tVec(i) = t;
end

%%
wVec = -diff(minYawVec)./diff(tVec);
