function [poseLog,tLog] = getPosesFromFixed(relPathFixed)
    %GETPOSESFROMFIXED
    %
    % [poseLog,tLog] = GETPOSESFROMFIXED(relPathFixed)
    %
    % relPathFixed -
    %
    % poseLog     - [nPoses,6]. [yxzrpy]
    % tLog         -
    
    yxzLog = [];
    rpyLog = [];
    tLog = [];
    nPoses = 0;
    
    fidFixed = fopen(relPathFixed,'r');
    
    line = fgetl(fidFixed);
    while ischar(line)
        c = strsplit(line);
        nPoses = nPoses+1;
        yxz = [str2num(c{5}), str2num(c{6}), str2num(c{7})];
        rpy = [str2num(c{27}), str2num(c{28}), str2num(c{29})];
        yxzLog(nPoses,:) = yxz;
        rpyLog(nPoses,:) = rpy;
        tLog(nPoses) = str2num(c{2});
        line = fgetl(fidFixed);
    end
    
    % subtract yxz0
    yxz0 = yxzLog(1,:);
    yxzLog = bsxfun(@minus,yxzLog,yxz0);
    
    % convert angles to rad
    rpyLog = deg2rad(rpyLog);
    
    poseLog = [yxzLog rpyLog];
end
