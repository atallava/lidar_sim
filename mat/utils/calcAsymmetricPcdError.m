function [errMean,errVar] = calcAsymmetricPcdError(pts1,pts2)
    [~,dists] = knnsearch(pts1,pts2);
    errMean = mean(dists);
    errVar = var(dists);
end