function posn = posnFromImuPose(imuPose)
%POSNFROMIMUPOSE
%
% posn = POSNFROMIMUPOSE(imuPose)
%
% imuPose - length 6 vector.
%
% posn    - [1,3] vector. [x,y,z] in world frame.

posn = [imuPose(2) imuPose(1) imuPose(3)];
end
