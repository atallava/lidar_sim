function numFrames = getNumFrames(dirName)
    % number of images in folder
    dirRes = dir(dirName);

    % hack. assuming end-1 is an image
    lastFrame = dirRes(end-1).name; 
    posn1 = strfind(lastFrame,'_');
    posn1 = posn1(end)+1;
    posn2 = strfind(lastFrame,'.jpg')-1;
    numFramesStr = lastFrame(posn1:posn2);
    numFrames = str2num(numFramesStr);
end