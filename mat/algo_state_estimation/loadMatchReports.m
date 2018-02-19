function [successVec,durationVec,inlierRatioVec,fitnessVec,dispArray]  = loadMatchReports(relPathFile)
%LOADMATCHREPORTS
%
% [successVec,durationVec,inlierRatioVec,fitnessVec,dispArray]  = LOADMATCHREPORTS(relPathFile)
%
% relPathFile    - string.
%
% successVec     - length nDisps vector.
% durationVec    - length nDisps vector.
% inlierRatioVec - length nDisps vector.
% fitnessVec     - length nDisps vector.
% dispArray]     - [nDisps,6] array.

fid = fopen(relPathFile,'r');
successVec = [];
durationVec = [];
inlerRatioVec = [];
fitnessVec = [];
dispArray = [];

count = 0;

line = fgetl(fid);
while ischar(line)
    count = count + 1;
    line = strtrim(line);
    c = strsplit(line, ' ');
    
    success = str2double(c{1});
    duration = str2double(c{2});
    if success
        inlierRatio = str2double(c{3});
        fitness = str2double(c{4});
        disp = zeros(1,7);
        for i = 1:7
            disp(i) = str2double(c{4+i});
        end
    else
        inlierRatio = 0;
        fitness = 0;
        disp = zeros(1,7);
    end
    
    successVec(count) = success;
    durationVec(count) = duration;
    inlierRatioVec(count) = inlierRatio;
    fitnessVec(count) = fitness;
    dispArray(count,:) = disp;
    
    line = fgetl(fid);
end
end