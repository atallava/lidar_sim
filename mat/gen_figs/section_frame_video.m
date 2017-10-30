%% helpers
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
startId = 8000;
endId = 9000;
skip = 1;
frameIds = startId:skip:endId;

%% frame times
relPathFrameTimes = 'frame_times';
load(relPathFrameTimes,'frameIdLog','frameTLog');
frameTimeFn = @(frameId) getTimeFromFrameId(frameId,frameIdLog,frameTLog);

%% frame rate
relPathVideo = 'tmp.avi';
outputVideo = VideoWrite(relPathVideo);
outputVideo.FrameRate = 18;
open(outputVideo);

%% make video
clockLocal = tic();
hWaitbar = waitbar(0,'progress');

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

    waitbar(frameCount/length(frameIds));
    waitbarTitle = sprintf('progress: %d/%d',frameCount,length(frameIds));
    setWaitbarTitle(hWaitbar,waitbarTitle);
end
compTime = toc(clockLocal);
fprintf('computation time: %.2fs\n',compTime);

%%
close(outputVideo);