function [poseLog,tLog] = getPosesFromFixed(relPathFixed)
%GETPOSESFROMFIXED 
% 
% [poseLog,tLog] = GETPOSESFROMFIXED(relPathFixed)
% 
% relPathFixed - 
% 
% poseLog     - [nPoses,6]. [xyzrpy]
% tLog         - 

xyzLog = [];
rpyLog = [];
tLog = [];
nPoses = 0;

fidFixed = fopen(relPathFixed,'r');

line = fgetl(fidFixed);
while ischar(line)
    c = strsplit(line);
    nPoses = nPoses+1;
    xyz = [str2num(c{5}), str2num(c{6}), str2num(c{7})];
    rpy = [str2num(c{27}), str2num(c{28}), str2num(c{29})];
    xyzLog(nPoses,:) = xyz;
    rpyLog(nPoses,:) = rpy;
    tLog(nPoses) = str2num(c{2});
    line = fgetl(fidFixed);
end

% subtract xyz0
xyz0 = xyzLog(1,:);
xyzLog = bsxfun(@minus,xyzLog,xyz0);

% convert angles to rad
rpyLog = deg2rad(rpyLog);

poseLog = [xyzLog rpyLog];
end
