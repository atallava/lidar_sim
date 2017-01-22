function posns = getPosnFromImuPose(poses)
    posns = [poses(:,2) poses(:,1) poses(:,3)];
end