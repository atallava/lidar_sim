function T_laser_world = getLaserTransfFromImuPose(pose,T_laser_imu)
T_imu_world = getImuTransfFromImuPose(pose);
T_laser_world = T_imu_world*T_laser_imu;

    % annoying convention
    y = pose(1);
    x = pose(2);
    z = pose(3);
    roll = pose(4);
    pitch = pose(5);
    yaw = pose(5);
    
    rollAxisAngle = [0 1 0 roll];
    pitchAxisAngle = [1 0 0 pitch];
    yawAxisAngle = [0 0 1 -yaw];
    
    R = vrrotvec2mat(yawAxisAngle)*vrrotvec2mat(pitchAxisAngle)*vrrotvec2mat(rollAxisAngle);
    T = eye(4,4);
    T(1:3,1:3) = R;
    T(1:3,4) = [x; y; z];
end