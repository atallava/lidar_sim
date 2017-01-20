function hfig = plotSimData(inputStruct)
    
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
        
        if isfield(ellipsoidData,'meanCell')
            meanCell = ellipsoidData.meanCell;
        else
            error('meanCell not a field in ellipsoidData.');
        end
        if isfield(ellipsoidData,'covMatCell')
            covMatCell = ellipsoidData.covMatCell;
        else
            error('covMatCell not a field in ellipsoidData.');
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
    
    if isfield(inputStruct,'simPts')
        plotSimPts = true;
        simPts = inputStruct.simPts;
    else
        plotSimPts = false;
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
            pts = genPtsRay(rayOrigin,rayDirns(i,:),10);
            plot3(pts(:,1),pts(:,2),pts(:,3),'g--');
            hold on;
        end
    end
    plot3(rayOrigin(1),rayOrigin(2),rayOrigin(3),'gx');
    
    % plot ellipsoids
    if plotEllipsoids
        nEllipses = length(meanCell);
        if useIntersectionFlag
            intersectedEllipses = sum(intersectionFlag,1);
        end
        for i = 1:nEllipses
            if useIntersectionFlag
                if ~intersectedEllipses(i)
                    continue;
                end
            end
            thisMean = meanCell{i};
            thisCovMat = covMatCell{i};
            
            [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean);
            surf(xEll,yEll,zEll,'facecolor','r','facealpha',0.2,'meshstyle','none');
            hold on;
        end
    end
    
    % plot sim pts
    if plotSimPts
        if useHitFlag
            ptPlotIds = logical(hitFlag);
        else
            ptPlotIds = logical(1:size(simPts,1));
        end
        plot3(simPts(ptPlotIds,1),simPts(ptPlotIds,2),simPts(ptPlotIds,3),'go','markerfacecolor','g');
    end
    
    %% bookkeeping
    axis equal;
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end