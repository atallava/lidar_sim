function [timestamps,TCell] = loadScanPoses(relPathScanPoses)
%LOADSCANPOSES
%
% [timestamps,TCell] = LOADSCANPOSES(relPathScanPoses)
%
% relPathScanPoses - string.
%
% timestamps       - nScans length vector.
% TCell            - nScans length cell array. TCell{i} is [4,4] array.

fid = fopen(relPathScanPoses,'r');
timestamps = [];
TCell = {};
nScans = 0;

line = fgetl(fid);

while ischar(line)
    nScans = nScans + 1;
    c = strsplit(strtrim(line));
    t_sec = str2double(c{1});
    t_nanosec = str2double(c{2});
    t = t_sec + t_nanosec*1e-9;
    
    T = str2double(c(3:end));
    T = reshape(T,4,4);
    
    timestamps(nScans) = t;
    TCell{nScans} = T;
    line = fgetl(fid);
end

end
