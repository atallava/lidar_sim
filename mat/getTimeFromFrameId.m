function t = getTimeFromFrameId(id)
    %GETTIMEFROMFRAMEID Eyeballing, use logged timestamps instead.
    %
    % t = GETTIMEFROMFRAMEID(id)
    %
    % id -
    %
    % t  -

    frameRate = 2.556e3/38384;
    offset = 0;
    t = offset+frameRate*id;
end