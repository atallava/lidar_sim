%% load section pts
relPathSectionPts = '../data/section_pts_04_world_frame_subsampled';
load(relPathSectionPts,'pts');

%% load imu poses
relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'poseLog','tLog');

%% laser calib
relPathLaserCalibParams = '../data/laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

%% section times

% section 3
% tStart = 1403045836.830185000;
% tEnd = 1403045902.775886000;

tStart = 1403045911.411350000;
tEnd = 1403046027.954617000;

tResn = 1; 
tKeypoints = tStart:tResn:tEnd;
nKeypoints = length(tKeypoints);

%% calculate camera params
cameraPositions = zeros(nKeypoints,3);
cameraTargets = zeros(nKeypoints,3);

for i = 1:length(tKeypoints)
t = tKeypoints(i);
poseId = indexOfNearestTime(t,tLog);
imuPose = poseLog(poseId,:);
T_imu_world = getImuTransfFromImuPose(imuPose);

[cameraPosition,cameraTarget] = getCameraParamsFromTransf(T_imu_world);

cameraPositions(i,:) = cameraPosition;
cameraTargets(i,:) = cameraTarget;
end

viewDirns = calcViewDirns(cameraPositions,cameraTargets);

%%
scatter3(pts(:,1),pts(:,2),pts(:,3),'b.');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); 
hold on;

quiver3(cameraPositions(:,1),cameraPositions(:,2),cameraPositions(:,3), ...
    viewDirns(:,1),viewDirns(:,2),viewDirns(:,3),'r');

%% write out
relPathOutCsv = '../data/section_04_camera_transfs.csv';
writeCameraTransforms(relPathOutCsv,tKeypoints-tStart,cameraPositions,viewDirns);

%%
relPathOutMat = '../data/section_04_camera_transfs';
save(relPathOutMat,'tStart','tKeypoints','cameraPositions','viewDirns');
 