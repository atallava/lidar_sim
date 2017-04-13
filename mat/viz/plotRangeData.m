function hfig = plotRangeData(inputStruct)
    
    %% unpack 
    
    %% rayData
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
        if isfield(rayData,'rayLengthToPlot')
            rayLengthToPlot = rayData.rayLengthToPlot;
        else
            rayLengthToPlot = 15;
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
    
    %% ellipsoidData
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
    
    %% triModelData
    if isfield(inputStruct,'triModelData')
        plotTri = true;
        triModelData = inputStruct.triModelData;
        
        if isfield(triModelData,'tri')
        else
            error('tri not a field in triModelData');
        end
        if isfield(triModelData,'ptsFit')
        else
            error('ptsFit not a field in triModelData');
        end
        
        if isfield(triModelData,'intersectionFlag')
            useIntersectionFlag = true;
            intersectionFlag = triModelData.intersectionFlag;
        else
            useIntersectionFlag = false;
        end
    else
        plotTri = false;
    end
    
    %% pts
    if isfield(inputStruct,'pts')
        plotPts = true;
        pts = inputStruct.pts;
        
        if ~exist('useHitFlag','var')
            useHitFlag = 0;
        end
    else
        plotPts = false;
    end
    
    %% plot
    hfig = figure(); 
    
    %% rays
    if plotRays
        nRays = size(rayDirns,1);
        for i = 1:nRays
            if useHitFlag
                if ~hitFlag(i)
                    continue
                end
            end
            rayPts = genPtsRay(rayOrigin,rayDirns(i,:),rayLengthToPlot);
%             plot3(rayPts(:,1),rayPts(:,2),rayPts(:,3),'r--');
            plot3(rayPts(:,1),rayPts(:,2),rayPts(:,3),'r','linewidth',2);
            hold on;
        end
        plot3(rayOrigin(1),rayOrigin(2),rayOrigin(3),'rx');
    end
    
    %% ellipsoids
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
            % transparencies not modulated with perm
            surf(xEll,yEll,zEll,'facecolor','g','facealpha',0.2,'meshstyle','none');
            hold on;
        end
    end
    
    %% triangles
    if plotTri
        if useIntersectionFlag
            intersectedTri = sum(intersectionFlag,1);
            triIdsToPlot = find(intersectedTri);
        else
            triIdsToPlot = 1:size(triModelData.tri,1);
        end
        
        mudBrownColor = [210 180 140]/255.0;
        trimesh(triModelData.tri(triIdsToPlot,:), ...
            triModelData.ptsFit(:,1),triModelData.ptsFit(:,2),triModelData.ptsFit(:,3),'edgecolor',mudBrownColor);
        hold on;
    end
    
    %% pts
    if plotPts
        if useHitFlag
            ptPlotIds = logical(hitFlag);
        else
            ptPlotIds = logical(1:size(pts,1));
        end
        scatter3(pts(ptPlotIds,1),pts(ptPlotIds,2),pts(ptPlotIds,3),'r.','markerfacecolor','r');
    end
    
    %% extra
    axis equal;
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end