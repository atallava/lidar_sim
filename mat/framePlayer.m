function segments = framePlayer(frameRelPathsPre,frameRelPathsPost,frameIds,getFrameTimeFromId,...
    xyLog,xyTimeLog)

%% set up figure

% only cam
% hfig = figure;
% hold on;

% cam + path

hfig = figure;
% subplot(2,1,1);
% hold on;

subplot(2,1,2);
plot(xyLog(:,1),xyLog(:,2));
hold on;
axis equal;

hCursor = plot(nan,nan,'ro');
set(hfig,'KeyPressFcn',@myKeyPress);

% increase fig size
figPosn = get(hfig,'Position');
figPosn(3) = figPosn(3)+300;
figPosn(4) = figPosn(4)+300;
set(hfig,'Position',figPosn);

%% define control keys
keys = struct(...
    'play','space',...
    'quit','q',...
    'longRwd','h','shortRwd','j',...
    'shortFwd','k','longFwd','l',...
    'print','p');

fprintf('%s : play/pause\n',keys.play);
fprintf('(%s,%s,%s,%s) : (<<,<,>,>>) when paused\n',...
    keys.longRwd,keys.shortRwd,keys.shortFwd,keys.longFwd);
fprintf('%s : quit\n',keys.quit);
fprintf('%s: print frame id\n',keys.print);

%% initialize
state = 0; 
segmentState = 'start';
segmentCount = 1;
segments = struct('startId',{},'endId',{},'comments',{});
segment = struct('startId',0,'endId',0,'comments','');


frameCount = 1;
shortStep = 1;
longStep = 5;
tPause = 5e-2;

%% player loop
while true
    switch state
        case 0
            % pause
            pause(tPause);
            continue;
        case 1
            % play
            dispFrame(frameCount);
            frameCount = frameCount+1;
            pause(tPause);
            continue;
        case -1
            % quit
            fprintf('Exit stream.\n');
            break;
        otherwise
            msg = sprintf('%s: state must be {-1,0,1}.\n',mfilename);
            error(msg);
    end
end

%% keypress callback
    function myKeyPress(hObj,event)
        switch event.Key
            case keys.play
                switch state
                    case 0
                        state = 1;
                    case 1
                        state = 0;
                    otherwise
                        error('state value has to be 0 or 1 when running.');
                end
            case keys.quit
                state = -1;
            case keys.longRwd
                % long rwd if paused
                if state == 0
                    frameCount = max(frameCount-longStep,1);
                    dispFrame(frameCount);
                end
            case keys.shortRwd
                % short rwd if paused
                if state == 0
                    frameCount = max(frameCount-shortStep,1);
                    dispFrame(frameCount);
                end
            case keys.shortFwd
                % short fwd if paused
                if state == 0
                    frameCount = max(frameCount+shortStep,1);
                    dispFrame(frameCount);
                end
            case keys.longFwd
                % long rew if paused
                if state == 0
                    frameCount = max(frameCount+longStep,1);
                    dispFrame(frameCount);
                end
            case keys.print
                % print frame id if paused
                if state == 0
                    frameId = frameIds(frameCount);
                    fprintf('Frame id: %d\n',frameId);
                    
                    % log interest segments
                    if strcmp(segmentState,'start')
                        segment.startId = frameId;
                        segmentState = 'end';
                    elseif strcmp(segmentState,'end')
                        segment.endId = frameId;
                        choice = input('Log segment? (y/n): \n','s');
                        if strcmp(choice,'y')
                            segment.comments = input('comments: \n','s');
                            segments(segmentCount) = segment;
                            segmentCount = segmentCount+1;
                        end
                        segmentState = 'start';
                    end
                end
            otherwise
                % do nothing
        end
    end

%% display frame implementation
    function dispFrame(frameCount)
        clear frame;
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
        
        % just the camera
%         figure(hfig);
%         imshow(panelFrame,'InitialMagnification',30);
%         title(num2str(frameId));

        % camera + path 
        frameTime = getFrameTimeFromId(frameId);
        xyId = indexOfNearestTime(frameTime,xyTimeLog);
        
        figure(hfig);
        subplot(2,1,1);
        subimage(panelFrame);
        title1 = sprintf('frame id: %d, time: %.2fs',frameId,frameTime);
        title(title1);
        
        subplot(2,1,2);
        xy = xyLog(xyId,:);
        set(hCursor,'XData',xy(1),'YData',xy(2));
        title2 = sprintf('path id: %d, time: %.2fs',xyId,frameTime);
        title(title2);
    end

end