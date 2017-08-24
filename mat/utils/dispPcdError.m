function dispPcdError(pts1,pts2)
%DISPPCDERROR
%
% DISPPCDERROR(pts1,pts2)
%
% pts1 -
% pts2 -

[errMean12,errVar12] = calcAsymmetricPcdError(pts1,pts2);
[errMean21,errVar21] = calcAsymmetricPcdError(pts2,pts1);
errMeanSym = mean([errMean12 errMean21]);

fprintf('error 12. mean: %.3f, var: %.3f\n',errMean12,errVar12);
fprintf('error 21. mean: %.3f, var: %.3f\n',errMean21,errVar21);
fprintf('error mean sym: %.3f\n',errMeanSym);
end