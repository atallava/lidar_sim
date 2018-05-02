function [timestamps,TCell] = loadDisps(relPathDisps)
%LOADDISPS
%
% [timestamps,TCell] = LOADDISPS(relPathDisps)
%
% relPathDisps -
%
% timestamps   -
% TCell        -

fid = fopen(relPathDisps,'r');
timestamps = [];
TCell = [];
nDisps = 0;

line = fgetl(fid);

while ischar(line)
    nDisps = nDisps + 1;
    line = strtrim(line);
    c = strsplit(line);
    tSecs = str2double(c{1});
    tNanosecs = str2double(c{2});
    t = tSecs + tNanosecs*1e-9;
    
    T = str2double(c(3:end));
    T = reshape(T,4,4);

    timestamps(nDisps) = t;
    TCell{nDisps} = T;
    line = fgetl(fid);
end
end