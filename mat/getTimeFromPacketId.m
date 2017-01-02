function t = getTimeFromFrameId(id)
    frameRate = 2.556e3/38384;
    offset = 0;
    t = offset+frameRate*id;
end