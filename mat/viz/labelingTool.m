function labeling = labelingTool(ptsCell,primitiveClasses,labelingData,imuData)
    %LABELINGTOOL
    %
    % labeling = LABELINGTOOL(ptsCell,primitiveClasses,relPathLabeling)
    %
    % ptsCell          - [1,nSegments] cell array. ptsCell{i} is 2d array.
    % primitiveClasses - [1,nClasses] cell array. primitiveClasses{i} is a
    % string.
    % relPathLabeling  - string. Save labeling to location.
    %
    % labeling         - [1,nSegments] vector.
    
    %% plot choices
    nClasses = length(primitiveClasses);
    nColorsToCycleThrough = 10;
    
    selectionColor = [1 0 0]; % red
    someDistinguishableColors = distinguishable_colors( ...
        nClasses+nColorsToCycleThrough,selectionColor);
    classColors = someDistinguishableColors(1:nClasses,:);
    cycleColors = someDistinguishableColors(nClasses+1:end,:);
    
    unlabeledMarker = '.';
    % todo: change this back to s
    labeledMarker = '.';
    
    shortStep = 1;
    longStep = 5;
    
    % todo. 8 is usual.
    legendFontSize = 30; 
    
    %% control keys
    keys = struct(...
        'select','space', ...
        'vis','v', ...
        'quit','q', ...
        'save','s', ...
        'spotlight','t', ...
        'dcSelect','m', ...
        'label','c', ...
        'longRwd','h','shortRwd','j', ...
        'shortFwd','k','longFwd','l');
    
    
    fprintf('%s : select/ unselect\n',keys.select);
    fprintf('%s: when selected, toggle visibility\n',keys.vis);
    fprintf('(%s,%s,%s,%s) : (<<,<,>,>>) when paused\n',...
        keys.longRwd,keys.shortRwd,keys.shortFwd,keys.longFwd);
    fprintf('%s : quit\n',keys.quit);
    fprintf('%s: save current labeling\n',keys.save);
    fprintf('%s: spotlight unlabeled\n',keys.spotlight);
    fprintf('%s: select segment where data cursor is currently\n',keys.dcSelect);
    fprintf('%s: open label box\n',keys.label);
    fprintf('1-%d: when segment is selected, label as\n',length(primitiveClasses));
    fprintf('0: when segment is selected, de-label\n');
    
    %% initialize
    nSegments = length(ptsCell);
    if ~isfield(labelingData,'loadPartialLabeling')
        labelingData.loadPartialLabeling = 0;
    end
    if labelingData.loadPartialLabeling
        load(labelingData.relPathPartialLabeling,'labeling');
    else
        labeling = zeros(1,nSegments);
    end
    
    selected = 0;
    currentSelectionId = 0;
    segmentColors = cell(1,nSegments);
    visibilityState = cell(1,nSegments);
    for i = 1:nSegments
        % if labeled, use class color
        if labeling(i)
            segmentColors{i} = ...
                classColors(labeling(i),:);
        else
            colorId = modN(i,nColorsToCycleThrough);
            segmentColors{i} = cycleColors(colorId,:);
        end
        
        visibilityState{i} = 'on';
    end
    
    
    ptsAll = [];
    ptsAllSegmentMap = [];
    for i = 1:length(ptsCell)
        thisPts = ptsCell{i};
        ptsAll = [ptsAll; thisPts];
        ptsAllSegmentMap = [ptsAllSegmentMap ...
            ones(1,size(thisPts,1))*i];
    end
       
    %% set up figure
    hfig = figure;
    hold on; axis equal;
    box on; grid off;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    
    scatterHandles = gobjects(1,nSegments);
     % todo. 20 is usual
     markerSizeData = 300;
    for i = 1:nSegments
        segmentPts = ptsCell{i};
        
        % if labeled, use apt marker
        if labeling(i)
            thisMarker = labeledMarker;
        else
            thisMarker = unlabeledMarker;
        end
        % todo: skipping!
        skip = 2;
        scatterHandles(i) = scatter3(segmentPts(1:skip:end,1),segmentPts(1:skip:end,2),segmentPts(1:skip:end,3),...
            'marker',thisMarker,'markerEdgeColor',segmentColors{i},'sizeData',markerSizeData);
    end
    
    set(hfig,'KeyPressFcn',@myKeyPress);
    
    % figure legend
    % plot one point, make invisible, legend for those only
    classColorMarkerHandles = gobjects(1,nClasses);
    legendEntries = cell(1,nClasses);
    pt = segmentPts(1,:);
    for i = 1:nClasses
        classColorMarkerHandles(i) = scatter3(pt(1),pt(2),pt(3),...
            'markerEdgeColor',classColors(i,:),'markerFaceColor',classColors(i,:),'visible','off');
        
        legendEntries{i} = sprintf('%d: %s',i, ...
            replaceUnderscoreWithSpace(primitiveClasses{i}));
    end
    hlegend = legend(classColorMarkerHandles,legendEntries);
    set(hlegend,'fontsize',legendFontSize);
    legendPosn = get(hlegend,'position');
    legendPosn = legendPosn + [1 1 0 0]*0.1;
    set(hlegend,'position',legendPosn);

    % figure tags
    tagLocations = zeros(nSegments,3);
    calcSegmentsTagLocations();
    % create tag handles
    tagHandles = gobjects(1,nSegments);
    % todo. 8 is usual
    tagFontSize = 20; 
    for i = 1:nSegments
        if labeling(i)
            tagText = num2str(labeling(i));
        else
            tagText = '';
        end
        tagHandles(i) = text(tagLocations(i,1),tagLocations(i,2),tagLocations(i,3),tagText);
        tagHandles(i).FontSize = tagFontSize;
    end
    
    %drawBasePlane();
    
    drawImuObbs();

    dcmObj = datacursormode(hfig);
    
    spotlightState = 0;
    
    %% callback
    function myKeyPress(hObj,event)
        numericCases = cell(1,nClasses+1);
        for j = 1:nClasses
            numericCases{j} = int2str(j);
        end
        numericCases{end} = int2str(0);
        switch event.Key
            case keys.select
                if ~selected
                    % pick an id at random
                    newSelectionId = randperm(nSegments,1);
                    updateCurrentSelection(newSelectionId);
                    selected = 1;
                else
                    set(scatterHandles(currentSelectionId),'markerEdgeColor', ...
                        segmentColors{currentSelectionId});
                    currentSelectionId = 0;
                    selected = 0;
                end
                
            case keys.vis
                if selected
                    if strcmp(visibilityState{currentSelectionId},'on')
                        visibilityState{currentSelectionId} = 'off';
                    else
                        visibilityState{currentSelectionId} = 'on';
                    end
                end
                
                
            case keys.spotlight
                if spotlightState
                    for i = 1:length(labeling)
                        if labeling(i)
                            set(scatterHandles(i),'visible',visibilityState{i});
                        end
                    end
                    spotlightState = 0;
                else
                    for i = 1:length(labeling)
                        if labeling(i)
                            set(scatterHandles(i),'visible','off');
                        end
                    end
                    spotlightState = 1;
                end
                
            case keys.shortFwd
                if ~selected
                    % pick an id at random
                    newSelectionId = randperm(nSegments,1);
                    updateCurrentSelection(newSelectionId);
                    selected = 1;
                else
                    newSelectionId = stepCircular(nSegments,currentSelectionId,shortStep);
                    updateCurrentSelection(newSelectionId);
                end
                
            case keys.longFwd
                if ~selected
                    % pick an id at random
                    newSelectionId = randperm(nSegments,1);
                    updateCurrentSelection(newSelectionId);
                    selected = 1;
                else
                    newSelectionId = stepCircular(nSegments,currentSelectionId,longStep);
                    updateCurrentSelection(newSelectionId);
                end
                
            case keys.shortRwd
                if ~selected
                    % pick an id at random
                    newSelectionId = randperm(nSegments,1);
                    updateCurrentSelection(newSelectionId);
                    selected = 1;
                else
                    newSelectionId = stepCircular(nSegments,currentSelectionId,-shortStep);
                    updateCurrentSelection(newSelectionId);
                end
                
            case keys.longRwd
                if ~selected
                    % pick an id at random
                    newSelectionId = randperm(nSegments,1);
                    updateCurrentSelection(newSelectionId);
                    selected = 1;
                else
                    newSelectionId = stepCircular(nSegments,currentSelectionId,-longStep);
                    updateCurrentSelection(newSelectionId);
                end
            
            case keys.label
                choice = inputdlg('label id:'); 
                
                if ~isempty(choice)
                    % choice is empty if click 'cancel' e.g.
                    choice = choice{1};
                    switch choice
                        case numericCases
                            if selected
                                classId = str2num(choice); classId = floor(classId);
                                if ~strcmp(choice,'0')
                                    % assign class
                                    labeling(currentSelectionId) = classId;
                                    % class color
                                    colorForClass = classColors(classId,:);
                                    segmentColors{currentSelectionId} = colorForClass;
                                    % labeled marker
                                    set(scatterHandles(currentSelectionId),'marker',labeledMarker);
                                    
                                    % tag
                                    set(tagHandles(currentSelectionId),'string',choice,'color',colorForClass);
                                else
                                    % de-assign class
                                    labeling(currentSelectionId) = 0;
                                    % set one of the cycle colors
                                    unlabeledColorId = randperm(nColorsToCycleThrough,1);
                                    unlabeledColor = cycleColors(unlabeledColorId,:);
                                    segmentColors{currentSelectionId} = unlabeledColor;
                                    % unlabeled marker
                                    set(scatterHandles(currentSelectionId),'marker',unlabeledMarker);
                                    
                                    % blank tag
                                    set(tagHandles(currentSelectionId),'string','');
                                end
                            end
                        otherwise
                            % do nothing
                    end
                end
                
            case keys.dcSelect
                dcInfo = getCursorInfo(dcmObj);
                dcPosn = dcInfo.Position;
                closestPtId = knnsearch(ptsAll,dcPosn);
                closestSegmentId = ptsAllSegmentMap(closestPtId);
                updateCurrentSelection(closestSegmentId);
                                
            case keys.save
                if isfield(labelingData,'relPathLabelingOut')
                    fprintf('saving labeling to %s...\n',labelingData.relPathLabelingOut);
                    save(labelingData.relPathLabelingOut,'labeling');
                else
                    msg = sprintf('%s: labelingData does not have field relPathLabelingOut\n', ...
                        mfilename);
                    error(msg); %#ok<SPERR>
                end
                
            case keys.quit
                if isfield(labelingData,'relPathLabelingOut')
                    fprintf('saving labeling to %s...\n',labelingData.relPathLabelingOut);
                    save(labelingData.relPathLabelingOut,'labeling');
                end
                close(hfig);

            otherwise
                % do nothing
        end
    end
    
    %% helpers
    function updateCurrentSelection(newSelectionId)
        % restore current selection's color
        % restore current selection's visibility state
        if currentSelectionId ~= 0
            set(scatterHandles(currentSelectionId),'markerEdgeColor',segmentColors{currentSelectionId});
            set(scatterHandles(currentSelectionId),'visible',visibilityState{currentSelectionId});
        end
        
        currentSelectionId = newSelectionId;
        set(scatterHandles(currentSelectionId),'markerEdgeColor',selectionColor);
        set(scatterHandles(currentSelectionId),'visible','on');
        selected = 1;
    end
    
    function calcSegmentsTagLocations()
        tagZOffset = 5;
        for i = 1:nSegments
            segmentPts = ptsCell{i};
            centroid = mean(segmentPts,1);
            zMax = max(segmentPts(:,3));
            tagLocations(i,:) = [centroid(1) centroid(2) zMax+tagZOffset];
        end
    end
    
    function drawBasePlane()
        obbAll = calcObb(ptsAll);
        obbAllVertices = getObbVertices(obbAll);
        xPlane = obbAllVertices(1:4,1);
        yPlane = obbAllVertices(1:4,2);
        zPlane = obbAllVertices(1:4,3);
        fill3(xPlane,yPlane,zPlane,[0 0 1],'faceAlpha',0.2,'edgealpha',0);
    end
    
    function drawImuObbs()
        % create a dummy obb
        dummyObb.center = [1 0 0];
        dummyObb.ax1 = [1 0]; dummyObb.ax2 = [0 1];
        dummyObb.extents = [-1 1; -1 1; -1 1];
        tImu = imuData.tExtents(1);
        
        while tImu < imuData.tExtents(2)
            imuPose = getImuPoseAtTime(imuData.poseLog,imuData.tLog,tImu);
            T_imu_world = getImuTransfFromImuPose(imuPose);
            obb_world = applyTransfToObb(dummyObb,T_imu_world);
            drawObb(hfig,obb_world);
            tImu = tImu+imuData.tResn;
        end
        
    end
end