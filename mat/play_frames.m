% 79,80,82: 10,12,2
camIds = [156579, 156580];
nCams = length(camIds);
frameRelPathsPre = cell(1,nCams);
frameRelPathsPost = cell(1,nCams);

for i = 1:nCams
    camId = camIds(i);
    dirRelPath = ['../data/taylorJune2014/lowres/AVTCamera_' num2str(camId)];
%     dirRelPath = ['~/taylor/lowres/AVTCamera_' num2str(camId)];
    frameNamePre = ['Camera_' num2str(camId)];
    frameNamePost = ['.jpg'];
    frameRelPathPre = [dirRelPath '/' frameNamePre];
    frameRelPathPost = frameNamePost;
    
    frameRelPathsPre{i} = frameRelPathPre;
    frameRelPathsPost{i} = frameRelPathPost;
end

numFrames = getNumFrames(dirRelPath); % assuming all have same number of frames

%% frame ids
startId = 1;
endId = 15000;
skip = 5;
frameIds = startId:skip:endId;

%% xy info
relPathXy = 'fixed_xy';
load(relPathXy,'xyLog','tLog');

xyLog(:,2) = -xyLog(:,2);

%%
segments = framePlayer(frameRelPathsPre,frameRelPathsPost,frameIds,@getTimeFromFrameId,...
    xyLog,tLog);