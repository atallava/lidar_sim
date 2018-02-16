% check output of humphrey's velocity integration 

%% load log
relPathFile = './check_vel_integration_log.txt';
[startTimes, endTimes, TDispCell] = loadCheckVelIntegrationLog(relPathFile);

%% load poses
relPathPoses = '../data/pose_log';
load(relPathPoses, 'poseLog', 'tLog');

%%
nLogs = length(startTimes);
TDispRefCell = cell(1,nLogs);
errVec = zeros(1,nLogs);
T_laser_imu = getTLaserToImu();

for i = 1:nLogs
    startTime = startTimes(i);
    endTime = endTimes(i);
    TDisp = TDispCell{i};
    
    poseStart = getImuPoseAtTime(poseLog, tLog, startTime);
    poseEnd = getImuPoseAtTime(poseLog, tLog, endTime);
    TStart_imu = getImuTransfFromImuPose(poseStart);
    TStart_laser = getLaserTransfFromImuPose(poseStart, T_laser_imu);
    TEnd_imu = getImuTransfFromImuPose(poseEnd);
    TEnd_laser = getLaserTransfFromImuPose(poseEnd, T_laser_imu);
    
    TDispRef = TStart_laser\TEnd_laser;
    
    errVec(i) = norm(TDispRef-TDisp, 'fro');
    TDispRefCell{i} = TDispRef;
end

