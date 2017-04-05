function imuPose = getImuPoseAtTime(poseLog,poseTLog,t)
    poseIndex = indexOfNearestTime(t,poseTLog);
    imuPose = poseLog(poseIndex,:);
end