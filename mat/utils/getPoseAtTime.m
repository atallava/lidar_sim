function pose = getPoseAtTime(poseLog,tLog,tQuery)
%GETPOSEATTIME Interpolates poseLog. Interpolation for angles might not be
% correct. So check before using for velocity calculation.
%
% pose = GETPOSEATTIME(poseLog,tLog,tQuery)
%
% poseLog - [nPoses,dimPose] array. imu poses.
% tLog    - length nPoses vector.
% tQuery  - scalar.
%
% pose    - [1,dimPose] array.

condn = (tQuery >= tLog(1)) && (tQuery <= tLog(end));
msg = sprintf('%s: tQuery not in range.\n',mfilename);
assert(condn,msg);

dimPose = size(poseLog,2);
pose = zeros(1,dimPose);
for i = 1:dimPose
    pose(i) = myInterp(poseLog(:,i),tLog,tQuery);
end

end

function vq = myInterp(v,x,xq)
flag = (x <= xq);
idx = sum(flag);

if (idx == length(v))
    vq = v(idx);
    return;
end

vq = v(idx) + (v(idx+1)-v(idx))*(xq-x(idx))/(x(idx+1)-x(idx));
end