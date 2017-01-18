% 79,80,82: 10,12,2
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

numFrames = getNumFrames(dirRelPath); % assuming all have same number of frames

%% specify frame ids to play
startId = 30670;
endId = 31330;
skip = 5;
frameIds = startId:skip:endId;

%% xy info
relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog');

yxLog = poseLog(:,1:2);
xyLog = fliplr(yxLog);

%% frame times
relPathFrameTimes = 'frame_times';
load(relPathFrameTimes,'frameIdLog','frameTLog');
frameTimeFn = @(frameId) getTimeFromFrameId(frameId,frameIdLog,frameTLog);

%%
segments = framePlayer(frameRelPathsPre,frameRelPathsPost,frameIds,frameTimeFn,...
    xyLog,tLog);