function hfig = plotRangeData(inputStruct)
%PLOTRANGEDATA 
% 
% hfig = PLOTRANGEDATA(inputStruct)
% 
% inputStruct - struct. fields ('rayData','ellipsoidData','triModelData','pts').
% 
% hfig        - figure handle.

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
        % note that the hit flag logic will be propagated to pts
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
        if isfield(ellipsoidData,'uniformAlpha')
            ellipsoidUniformAlpha = ellipsoidData.uniformAlpha;
        else
            ellipsoidUniformAlpha = false;
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
        
        if isfield(triModelData,'uniformAlpha')
            triUniformAlpha = triModelData.uniformAlpha;
        else
            triUniformAlpha = false;
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
        plot3(rayOrigin(1),rayOrigin(2),rayOrigin(3),'rx','linewidth',0.5);
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
            thisHitProb = ellipsoidModels(i).hitProb;
            
            [xEll,yEll,zEll] = genSurfXyzEllipse(thisCovMat,thisMean);
            
            if ellipsoidUniformAlpha
                thisAlpha = 0.5;
            else
                thisAlpha = mapHitProbToAlpha(thisHitProb);
            end
            
            surf(xEll,yEll,zEll,'facecolor','g','meshstyle','none', ...
                'facealpha',thisAlpha);
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
        
        if triUniformAlpha
            faceVertexAlpha = ones(size(triModelData.tri,1),1)*0.5;
        else
            faceVertexAlpha = flipVecToColumn(mapHitProbToAlpha(triModelData.hitProbVec));
        end
        
        mudBrownColor = [210 180 140]/255.0;
        saddleBrownColor = [139 69 19]/255.0;
        trimesh(triModelData.tri(triIdsToPlot,:), ...
            triModelData.ptsFit(:,1),triModelData.ptsFit(:,2),triModelData.ptsFit(:,3), ...
            'edgecolor',saddleBrownColor,'facecolor',mudBrownColor, ...
            'facealpha','flat','FaceVertexAlphaData',faceVertexAlpha);
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

%% helpers
function transparency = mapHitProbToAlpha(hitProb)
    alphaLow = 0;
    alphaHigh = 0.5;
    transparency = alphaLow + hitProb.*alphaHigh;
end
