function imuPose = getImuPoseAtTime(poseLog,poseTLog,t)
    %GETIMUPOSEATTIME
    %
    % imuPose = GETIMUPOSEATTIME(poseLog,poseTLog,t)
    %
    % poseLog  -
    % poseTLog -
    % t        -
    %
    % imuPose  -
    
    poseIndex = indexOfNearestTime(t,poseTLog);
    imuPose = poseLog(poseIndex,:);
end