function ellipsoids2 = applyTransfToEllipsoids(ellipsoids1,T_1_to_2)
    %APPLYTRANSFTOELLIPSOIDS
    %
    % ellipsoids2 = APPLYTRANSFTOELLIPSOIDS(ellipsoids1,T_1_to_2)
    %
    % ellipsoids1 - struct array.
    % T_1_to_2    - [4,4] array.
    %
    % ellipsoids2 - struct array.
    
    ellipsoids2 = ellipsoids1;
    muMat1 = getEllipsoidCenters(ellipsoids1);
    muMat2 = applyTransf(muMat1,T_1_to_2);
    nEllipsoids = length(ellipsoids1);
    for i = 1:nEllipsoids
        ellipsoids2(i).mu = muMat2(i,:);
        ellipsoids2(i).covMat = applyTransfToCov(ellipsoids2(i).covMat,T_1_to_2);
    end
end