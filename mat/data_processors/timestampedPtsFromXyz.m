function [tLog,pts] = timestampedPtsFromXyz(relPathFile)
    fid = fopen(relPathFile,'r');
    pts = [];
    tLog = [];
    count = 0;
    line = fgetl(fid);
    while ischar(line)
        c = strsplit(line);
        timestampSec = str2num(c{2});
        timestampNanosec = str2num(c{3});
        timestamp = timestampSec + timestampNanosec*1e-9;
        x = str2num(c{4}); y = str2num(c{5}); z = str2num(c{6});
        
        count = count+1;
        pts(count,:) = [x y z];
        tLog(count) = timestamp;
        line = fgetl(fid);
    end
end