camIds = [156579, 156580];
nCams = length(camIds);
frameRelPathsPre = cell(1,nCams);
frameRelPathsPost = cell(1,nCams);

for i = 1:nCams
    camId = camIds(i);
    dirRelPath = ['../../data/taylorJune2014/lowres/AVTCamera_' num2str(camId)];
    frameNamePre = ['Camera_' num2str(camId)];
    frameNamePost = ['.jpg'];
    frameRelPathPre = [dirRelPath '/' frameNamePre];
    frameRelPathPost = frameNamePost;
    
    frameRelPathsPre{i} = frameRelPathPre;
    frameRelPathsPost{i} = frameRelPathPost;
end

%% specify frame ids to play
startId = 9200;
endId = 11000;
skip = 5;
frameIds = startId:skip:endId;

%% frame times
relPathFrameTimes = 'frame_times';
load(relPathFrameTimes,'frameIdLog','frameTLog');
frameTimeFn = @(frameId) getTimeFromFrameId(frameId,frameIdLog,frameTLog);

%%
outputVideo = VideoWriter('section_04.avi');
outputVideo.FrameRate = 10;
open(outputVideo);

%%
clockLocal = tic();
for frameCount = 1:length(frameIds)
    clear panelFrame
    frameId = frameIds(frameCount);
    nCams = length(frameRelPathsPre);
    frames = cell(1,nCams);
    for i = 1:nCams
        frameRelPathPre = frameRelPathsPre{i};
        frameRelPathPost = frameRelPathsPost{i};
        frameRelPath = [frameRelPathPre '_' sprintf('%06d',frameId) frameRelPathPost];
        frames{i} = imread(frameRelPath);
        if i == 1
            panelFrame = frames{i};
        else
            panelFrame = cat(2,panelFrame,frames{i});
        end
    end
    % magic
    panelFrame = imadjust(panelFrame, [0 0 0; 0.25 0.25 0.25],[]);
    image(panelFrame);
    pause(0.5);
    axis equal;
    set(gca,'visible','off');
    set(gcf,'visible','off');
    writeVideo(outputVideo,panelFrame);
end
compTime = toc(clockLocal);
fprintf('computation time: %.2fs\n',compTime);

%%
close(outputVideo);