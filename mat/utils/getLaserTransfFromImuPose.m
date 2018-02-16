function T_laser_world = getLaserTransfFromImuPose(pose,T_laser_imu)
%GETLASERTRANSFFROMIMUPOSE
%
% T_laser_world = GETLASERTRANSFFROMIMUPOSE(pose,T_laser_imu)
%
% pose          - length 6 vector.
% T_laser_imu   - [4,4] array.
%
% T_laser_world - [4,4] array.

T_imu_world = getImuTransfFromImuPose(pose);
T_laser_world = T_imu_world*T_laser_imu;
end