function labeling = labelingTool(ptsCell,primitiveClasses,relPathLabeling)
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
    labeledMarker = 's';
    
    shortStep = 1;
    longStep = 5;
    
    legendFontSize = 8;
    
    %% control keys
    keys = struct(...
        'select','space', ...
        'quit','q',...
        'longRwd','h','shortRwd','j',...
        'shortFwd','k','longFwd','l');
    
    
    fprintf('%s : select/ unselect\n',keys.select);
    fprintf('(%s,%s,%s,%s) : (<<,<,>,>>) when paused\n',...
        keys.longRwd,keys.shortRwd,keys.shortFwd,keys.longFwd);
    fprintf('%s : quit\n',keys.quit);
    fprintf('1-%d: when segment is selected, label as\n',length(primitiveClasses));
    fprintf('0: when segment is selected, de-label\n');
    
    %% set up figure
    hfig = figure;
    hold on; axis equal;
    box on; grid on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    
    nSegments = length(ptsCell);
    labeling = zeros(1,nSegments);
    scatterHandles = gobjects(1,nSegments);
    for i = 1:nSegments
        segmentPts = ptsCell{i};
        colorId = modN(i,nColorsToCycleThrough);
        segmentPtsColor = cycleColors(colorId,:);
        
        scatterHandles(i) = scatter3(segmentPts(:,1),segmentPts(:,2),segmentPts(:,3),...
            'marker',unlabeledMarker,'markerEdgeColor',segmentPtsColor);
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
        
        legendEntries{i} = sprintf('%d: %s',i,primitiveClasses{i});
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
    for i = 1:nSegments
        tagHandles(i) = text(tagLocations(i,1),tagLocations(i,2),tagLocations(i,3),'');
    end
    
    %% initialize callback state
    selected = 0;
    currentSelectionId = 0;
    colorBackup = zeros(1,3);
    
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
                    set(scatterHandles(currentSelectionId),'markerEdgeColor',colorBackup);
                    colorBackup = [];
                    currentSelectionId = 0;
                    selected = 0;
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
                
            case numericCases
                if selected
                    classId = str2num(event.Key); classId = floor(classId);
                    if ~strcmp(event.Key,'0')
                        % assign class
                        labeling(currentSelectionId) = classId;
                        % class color
                        colorForClass = classColors(classId,:);
                        colorBackup = colorForClass;
                        % labeled marker
                        set(scatterHandles(currentSelectionId),'marker',labeledMarker);
                        
                        % tag
                        set(tagHandles(currentSelectionId),'string',event.Key,'color',colorForClass);
                    else
                        % de-assign class
                        labeling(currentSelectionId) = 0;
                        % set one of the cycle colors
                        unlabeledColorId = randperm(nColorsToCycleThrough,1);
                        unlabeledColor = cycleColors(unlabeledColorId,:);
                        colorBackup = unlabeledColor;
                        % unlabeled marker
                        set(scatterHandles(currentSelectionId),'marker',unlabeledMarker);
                        
                        % blank tag
                        set(tagHandles(currentSelectionId),'string','');
                    end
                end
            case keys.quit
                save(relPathLabeling,'labeling');
                close(hfig);
            otherwise
                % do nothing
        end
    end
    
    %% helpers
    function updateCurrentSelection(newSelectionId)
        % restore current selection's color
        if currentSelectionId ~= 0
            set(scatterHandles(currentSelectionId),'markerEdgeColor',colorBackup);
        end
        
        currentSelectionId = newSelectionId;
        colorBackup = get(scatterHandles(currentSelectionId), ...
            'markerEdgeColor');
        set(scatterHandles(currentSelectionId),'markerEdgeColor',selectionColor);
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
end