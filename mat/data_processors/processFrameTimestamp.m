function [frameIdLog,frameTLog] = processFrameTimestamp(relPathFrameTimestamp)
    fid = fopen(relPathFrameTimestamp,'r');
    dispFlag = 1;
    frameTLog = [];
    frameIdLog = [];
    count = 0;
    line = fgetl(fid); % ignore header
    line = fgetl(fid);
    while ischar(line)
        c = strsplit(line,',');
        frameId = getFrameIdFromFrameFilename(c{1});
        frameT = str2num(c{2});
        
        count = count+1;
        frameIdLog(count) = frameId;
        frameTLog(count) = frameT;
        
        if dispFlag
            if mod(frameId,1000) == 0
                fprintf('%s: frame id: %d\n',mfilename,frameId);
            end
        end
        line = fgetl(fid);
    end
end