function t = getTimeFromFrameId(frameId,frameIdLog,frameTLog)
    %GETTIMEFROMFRAMEID
    %
    % t = GETTIMEFROMFRAMEID(frameId,frameIdLog,frameTLog)
    %
    % frameId    -
    % frameIdLog -
    % frameTLog  -
    %
    % t          -
    
    t = frameTLog(frameId == frameIdLog);
end