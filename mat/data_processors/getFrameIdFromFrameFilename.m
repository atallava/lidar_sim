function frameId = getFrameIdFromFrameFilename(frameFilename)
    c1 = strsplit(frameFilename,'_');
    c2 = strsplit(c1{3},'.');
    frameId = str2num(c2{1});
end