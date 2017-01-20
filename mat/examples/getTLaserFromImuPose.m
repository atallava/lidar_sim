function TLaser = getTLaserFromImuPose(imuPose,TLaserImu)
    TImuWorld = getImuTransfFromPose(imuPose);
    TLaser = TLaserImu*TImuWorld;
end