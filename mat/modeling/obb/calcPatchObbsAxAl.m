function [obbCell,obbPtsCell] = calcPatchObbsAxAl(pts)
    %CALCPATCHOBBSAXAL
    %
    % [obbCell,obbPtsCell] = CALCPATCHOBBSAXAL(pts)
    %
    % pts        -
    %
    % obbCell    -
    % obbPtsCell -
    
    % xy-frame
    ptsXy = pts(:,1:2);
    frameCenter = mean(ptsXy,1);
    ptsXyCentered = bsxfun(@minus,ptsXy,frameCenter);
   
    frameExtents = zeros(2,2);
    lowLim = 1e-2;
    highLim = 1-lowLim;
    for i = 1:2
        frameExtents(i,1) = quantile(ptsXyCentered(:,i),lowLim);
        frameExtents(i,2) = quantile(ptsXyCentered(:,i),highLim);
    end
    
    globalZExtents = calcGlobalZExtents(pts);
   
    % grid
    nodeResn = 1;
    numXNodes = ceil((frameExtents(1,2)-frameExtents(1,1))/nodeResn);
    numYNodes = ceil((frameExtents(2,2)-frameExtents(2,1))/nodeResn);
    xNodes = [0:(numXNodes-1)]*nodeResn + 0.5*nodeResn + frameCenter(1)+frameExtents(1,1);
    yNodes = [0:(numYNodes-1)]*nodeResn + 0.5*nodeResn + frameCenter(2)+frameExtents(2,1);
    [xGridMat,yGridMat] = meshgrid(xNodes,yNodes);
    xGrid = xGridMat(:);
    yGrid = yGridMat(:);
    
    % decide which to keep 
    gridOccupancyCount = zeros(numYNodes,numXNodes);
    gridOccupancyPtIds = cell(numYNodes,numXNodes);
    for i = 1:size(pts,1)
        pt = ptsXy(i,:);
        if checkInFrame(pt)
            idX = ceil((pt(1)-(frameCenter(1)+frameExtents(1,1)))/nodeResn);
            idY = ceil((pt(2)-(frameCenter(2)+frameExtents(2,1)))/nodeResn); 
            gridOccupancyCount(idY,idX) = ...
                gridOccupancyCount(idY,idX)+1;
            gridOccupancyPtIds{idY,idX} = ...
                [gridOccupancyPtIds{idY,idX} i];
        end
    end
    minOccupancy = 10;
    occupancyFlagMat = gridOccupancyCount >= minOccupancy;
    occupancyFlag = occupancyFlagMat(:);
    
    %% obb for each grids
    gridIds = find(occupancyFlag);
    nObbs = sum(occupancyFlag);
    obbCell = cell(1,nObbs);
    obbPtsCell = cell(1,nObbs);
    for i = 1:nObbs
        gridId = gridIds(i);
        [gridRow,gridCol] = ind2sub(size(gridOccupancyPtIds),gridId);
        ptsInGridIds = gridOccupancyPtIds{gridRow,gridCol};
        ptsInGrid = pts(ptsInGridIds,:);
        zVec = ptsInGrid(:,3);
        zVecCentered = zVec-mean(zVec);
        obb.center = [xGrid(gridId) yGrid(gridId) mean(zVec)];
        
        
        obb.ax1 = [1 0];
        obb.ax2 = [0 1];
        
        obb.extents(1,1) = -0.5*nodeResn;
        obb.extents(1,2) = 0.5*nodeResn;
        obb.extents(2,1) = -0.5*nodeResn;
        obb.extents(2,2) = 0.5*nodeResn;
        obb.extents(3,1) = ...
            max(quantile(zVecCentered,lowLim),globalZExtents(1));
        obb.extents(3,2) = ...
            min(quantile(zVecCentered,highLim),globalZExtents(2));
        
        obbCell{i} = obb;
        obbPtsCell{i} = ptsInGrid;
    end

    %% helpers
    function res = checkInFrame(pt)
        condn1 = frameCenter(1)+frameExtents(1,1) <= pt(1);
        condn2 = pt(1) <= frameCenter(1)+frameExtents(1,2);
        condn3 = frameCenter(2)+frameExtents(2,1) <= pt(2);
        condn4 = pt(2) <= frameCenter(2)+frameExtents(2,2);
        res = condn1 & condn2 & condn3 & condn4;
    end
    
    function zExtents = calcGlobalZExtents(pts)
        zVec = pts(:,3)-mean(pts(:,3));
        zExtents(1) = quantile(zVec,lowLim);
        zExtents(2) = quantile(zVec,highLim);
    end
end