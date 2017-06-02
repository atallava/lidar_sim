function line = convertEllipsoidModelToStr(ellipsoidModel)
    %CONVERTELLIPSOIDMODELTOSTR
    %
    % line = CONVERTELLIPSOIDMODELTOSTR(ellipsoidModel)
    %
    % ellipsoidModel -
    %
    % line           -
    
    mu = ellipsoidModel.mu;
    line = '';
    for j = 1:3
        line = [line ' ' num2str(mu(j))];
    end
    covMat = ellipsoidModel.covMat;
    for j = 1:9
        line = [line ' ' num2str(covMat(j))];
    end
    line = [line ' ' num2str(ellipsoidModel.hitProb)];
    
    % remove space at start
    line(1) = [];
end