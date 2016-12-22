camId = 156579;
dirName = ['lowres/AVTCamera_' num2str(camId)];
frameNamePre = ['Camera_' num2str(camId)];
frameNamePost = ['.jpg'];

numFrames = getNumFrames(dirName);

%%
startId = 1;
endId = numFrames;
skip = 10;
frameIds = startId:skip:endId;
frameIds = [1 1000];

for frameId = frameIds
    frameName = [dirName '/' frameNamePre '_' ...
        sprintf('%06d',frameId) frameNamePost];
    frame = imread(frameName);
    hframe = image(frame);
    hold on;
    title(num2str(frameId));
    waitforbuttonpress;
end
