function features = calcFeaturesForClassification(pts,optionStruct)
    
    clockLocal = tic();

    %% unpack options
    if isfield(optionStruct,'searchRadius')
        searchRadius = optionStruct.searchRadius;
    else
        searchRadius = 0.6; % from munoz
    end
    if isfield(optionStruct,'minNbrs')
        minNbrs = optionStruct.minNbrs;
    else
        minNbrs = 5;
    end
    if isfield(optionStruct,'defaultSpectralFeatures')
        defaultSpectralFeatures = optionStruct.defaultSpectralFeatures;
    else
        defaultSpectralFeatures = ones(1,3); % perfect sphere
    end
    if isfield(optionStruct,'defaultDirnFeatures')
        defaultDirnFeatures = optionStruct.defaultDirnFeatures;
    else
        defaultDirnFeatures = [0 1 1 0]; % horizontal tangent, vertical normal
    end
    if isfield(optionStruct,'xyBoxSide')
        xyBoxSide = optionStruct.xyBoxSide;
    else
        xyBoxSide = 0.15;
    end
    if isfield(optionStruct,'dispFlag')
        dispFlag = optionStruct.dispFlag;
    else
        dispFlag = 0;
    end

    %% calc features
    nPts = size(pts,1);
    
    idx = rangesearch(pts,pts,searchRadius);
    covMatCell = cell(1,nPts);
    spectralFeatures = zeros(nPts,3);
    dirnFeatures = zeros(nPts,4);
    heightFeatures = zeros(nPts,1);
    fewNbrsFlag = zeros(1,nPts);
    
    % todo: all pts
    for i = 1:1e3%nPts
        nbrIdx = idx{i};
        nbrPts = pts(nbrIdx,:);
        if size(nbrPts,1) < minNbrs
            fewNbrsFlag(i) = 1;
            spectralFeatures(i,:) = defaultSpectralFeatures;
            dirnFeatures(i,:) = defaultDirnFeatures;
        else
            covMat = cov(nbrPts);
            [spectralFeatures(i,:),dirnFeatures(i,:)] = ...
                calcFeaturesFromCovMat(covMat);
            covMatCell{i} = covMat;
        end
        
        ptsInCylinder = getPtsInCylinder(pts,pts(i,:),xyBoxSide);
        heightFeatures(i,:) = calcHeightFeatures(ptsInCylinder,pts(i,:));
    end
    
    bias = ones(nPts,1);
    
    features = [spectralFeatures dirnFeatures heightFeatures bias];
    
    % todo: scale features?
    compTime = toc(clockLocal);
    if dispFlag
        fprintf('comp time: %.2fs\n',compTime);
    end
    
end

%% helpers
function [spectralFeatures,dirnFeatures] = calcFeaturesFromCovMat(covMat)
    [V,D] = eig(covMat);
    
    eigVals = diag(D);
    spectralFeatures = [eigVals(1) eigVals(1)-eigVals(2) eigVals(2)-eigVals(3)];
    
    zVec = [0; 0; 1];
    tangentVec = V(:,1);
    tangentVec = tangentVec/norm(tangentVec);
    cTangent = dot(tangentVec,zVec);
    sTangent = sqrt(1-cTangent^2);
    normalVec = V(:,3);
    normalVec = normalVec/norm(normalVec);
    cNormal = dot(normalVec,zVec);
    sNormal = sqrt(1-cNormal^2);
    dirnFeatures = [cTangent sTangent cNormal sNormal];
end

function ptsInCylinder = getPtsInCylinder(pts,pt,boxSide)
    obb.center = pt;
    obb.ax1 = [1 0];
    obb.ax2 = [0 1];
    obb.extents = [-1 1; -1 1]*boxSide;
    obb.extents(3,:) = [-1e2 1e2]; % large in the z direction
    
    flag = checkPtsInObb(pts,obb);
    ptsInCylinder = pts(flag,:);
end

function heightFeatures = calcHeightFeatures(ptsInCylinder,pt)
    if isempty(ptsInCylinder)
        heightFeatures = pt(3);
    else
        heightFeatures = pt(3)-min(ptsInCylinder(:,3));
    end
end