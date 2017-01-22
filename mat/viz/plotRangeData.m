function hfig = plotRangeData(inputStruct)
    
    %% unpack 
    if isfield(inputStruct,'rayData')
        plotRays = true;
        rayData = inputStruct.rayData;
        
        if isfield(rayData,'rayOrigin')
            rayOrigin = rayData.rayOrigin;
        else
            error('rayOrigin not a field in rayData.');
        end
        if isfield(rayData,'rayDirns')
            rayDirns = rayData.rayDirns;
        else
            error('rayDirns not a field in rayData.');
        end
        % note that the hit flag logic will be propagated to sim pts
        if isfield(rayData,'hitFlag')
            useHitFlag = true;
            hitFlag = rayData.hitFlag;
        else
            useHitFlag = false;
        end
    else
        plotRays = false;
    end
    
    if isfield(inputStruct,'ellipsoidData')
        plotEllipsoids = true;
        ellipsoidData = inputStruct.ellipsoidData;
        
        if isfield(ellipsoidData,'ellipsoidModels')
            ellipsoidModels = ellipsoidData.ellipsoidModels;
        else
            error('ellipsoidModels not a field in ellipsoidData.');
        end
        if isfield(ellipsoidData,'intersectionFlag')
            useIntersectionFlag = true;
            intersectionFlag = ellipsoidData.intersectionFlag;
        else
            useIntersectionFlag = false;
        end
    else
        plotEllipsoids = false;
    end
    
    if isfield(inputStruct,'pts')
        plotPts = true;
        pts = inputStruct.pts;
    else
        plotPts = false;
    end
    
    %% plot
    hfig = figure(); 
    
    % plot rays
    if plotRays
        nRays = size(rayDirns,1);
        for i = 1:nRays
            if useHitFlag
                if ~hitFlag(i)
                    continue
                end
            end
            % todo: this length can be the max laser length
            rayPts = genPtsRay(rayOrigin,rayDirns(i,:),50);
            plot3(rayPts(:,1),rayPts(:,2),rayPts(:,3),'g--');
            hold on;
        end
    end
    plot3(rayOrigin(1),rayOrigin(2),rayOrigin(3),'gx');
    
    % plot ellipsoids
    if plotEllipsoids
        nEllipses = length(ellipsoidModels);
        if useIntersectionFlag
            intersectedEllipses = sum(intersectionFlag,1);
        end
        for i = 1:nEllipses
            if useIntersectionFlag
                if ~intersectedEllipses(i)
                    continue;
                end
            end
            thisMean = ellipsoidModels(i).mu;
            thisCovMat = ellipsoidModels(i).covMat;
            
            [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean);
            surf(xEll,yEll,zEll,'facecolor','r','facealpha',0.2,'meshstyle','none');
            hold on;
        end
    end
    
    % plot sim pts
    if plotPts
        if useHitFlag
            ptPlotIds = logical(hitFlag);
        else
            ptPlotIds = logical(1:size(pts,1));
        end
        scatter3(pts(ptPlotIds,1),pts(ptPlotIds,2),pts(ptPlotIds,3),'go','markerfacecolor','g');
    end
    
    %% bookkeeping
    axis equal;
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end