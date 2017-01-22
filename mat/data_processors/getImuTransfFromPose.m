function T = getImuTransfFromPose(pose)
    % pose = [yxzrpy]
    
    x = pose(2);
    y = pose(1);
    z = pose(3);
    roll = pose(4);
    pitch = pose(5);
    yaw = pose(6);
    % this comes from cetin
    rollAngle = [0 1 0 roll];
    pitchAngle = [1 0 0 pitch];
    yawAngle = [0 0 1 -yaw];
    
    rotMat = vrrotvec2mat(yawAngle)*vrrotvec2mat(pitchAngle)*vrrotvec2mat(rollAngle);
    
%     T = zeros(4,4);
    T(1:3,1:3) = rotMat; 
    T(:,4) = [x; y; z];
    T(4,:) = [0 0 0 1];
end