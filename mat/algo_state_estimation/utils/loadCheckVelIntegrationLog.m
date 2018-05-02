function [startTimes, endTimes, TDispCell] = loadCheckVelIntegrationLog(relPathFile)
%LOADCHECKVELINTEGRATIONLOG
%
% [startTimes, endTimes, TDispCell] = LOADCHECKVELINTEGRATIONLOG(relPathFile)
%
% relPathFile -
%
% startTimes  -
% endTimes    -
% TDispCell   -

fid = fopen(relPathFile, 'r');
startTimes = [];
endTimes = [];
TDispCell = {};

count = 1;
line = fgetl(fid);
while ischar(line)
    c = strsplit(line);
    startTimeSecs = str2double(c{1});
    startTimeNsecs = str2double(c{2});
    startTime = startTimeSecs + startTimeNsecs*1e-9;
    
    endTimeSecs = str2double(c{3});
    endTimeNsecs = str2double(c{4});
    endTime = endTimeSecs + endTimeNsecs*1e-9;
    
    T = zeros(4,4);
    
    for col = 1:4
        for row = 1:4
            id = 4 + (col-1)*4 + row;
            T(row,col) = str2double(c{id});
        end
    end

    startTimes(count) = startTime;
    endTimes(count) = endTime;
    TDispCell{count} = T;
    count = count+1;
    line = fgetl(fid);
end
end