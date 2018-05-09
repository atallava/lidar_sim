function [posnError, rotnError] = calcPosesError(TCell1, TCell2)
%CALCPOSESERROR
%
% [posnError, rotnError] = CALCPOSESERROR(TCell1, TCell2)
%
% TCell1    -
% TCell2    -
%
% posnError -
% rotnError -

% todo: hack for mm_sim
nPoses = min(length(TCell1), length(TCell2));
posnErrorVec = zeros(1, nPoses);
rotnErrorVec = zeros(1, nPoses);

for i = 1:nPoses
    T1 = TCell1{i};
    T2 = TCell2{i};
    t1 = T1(1:3, 4); t2 = T2(1:3, 4);
    R1 = T1(1:3, 1:3); R2 = T2(1:3, 1:3);
    posnErrorVec(i) = norm(t1-t2);
    R_2_1 = R1\R2;
    axang = rotm2axang(R_2_1);
    rotnErrorVec(i) = abs(axang(end));
end

posnError = mean(posnErrorVec);
rotnError = mean(rotnErrorVec);
end