function [timestamps,dispArray] = loadGuessDisps(relPathGuessDisps)
%LOADSCANPOSES
%
% [timestamps,TCell] = LOADSCANPOSES(relPathScanPoses)
%
% relPathScanPoses - string.
%
% timestamps       - nScans length vector.
% TCell            - nScans length cell array. TCell{i} is [4,4] array.

fid = fopen(relPathGuessDisps,'r');
timestamps = [];
dispArray = [];
count = 0;

line = fgetl(fid);

while ischar(line)
    count = count + 1;
    c = strsplit(strtrim(line));
    t_sec = str2double(c{1});
    t_nanosec = str2double(c{2});
    t = t_sec + t_nanosec*1e-9;
    
    disp = str2double(c(3:end));
    
    timestamps(count) = t;
    dispArray(count,:) = disp;
    line = fgetl(fid);
end

end
