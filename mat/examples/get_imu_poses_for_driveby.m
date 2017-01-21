% validation dataset
relPathDriveby = 'section_08_driveby';
load(relPathDriveby,'pts','ptsTLog');
scanPts = pts;
scanPtsTLog = ptsTLog;

relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog');
poseTLog = tLog;

%%
nScanPts = size(scanPts,1);
imuPoses = zeros(nScanPts,6);
for i = 1:nScanPts
    t = scanPtsTLog(i);
    poseIndex = indexOfNearestTime(t,poseTLog);
    imuPose = poseLog(poseIndex,:);
    imuPoses(i,:) = imuPose;
end

%%
relPathOutput = [relPathDriveby '_poses'];
save(relPathOutput,'imuPoses','scanPtsTLog');
