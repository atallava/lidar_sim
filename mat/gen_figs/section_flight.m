%% load section pts
relPathSectionPts = '../data/section_pts_03_world_frame_subsampled_1e5';
load(relPathSectionPts,'pts');

%% load imu poses
relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'poseLog','tLog');

%% section times
tStart = 1403045836.830185000;
tEnd = 1403045902.775886000;
tResn = 5; 
tKeypoints = tStart:tResn:tEnd;

% go to those camera poses to see if they make sense
for i = 1%:length(tKeypoints)
t = tKeypoints(i);
poseId = indexOfNearestTime(t,tLog);
imuPose = poseLog(poseId,:);

end

% write out

% perform the wiring from sky to imu start

