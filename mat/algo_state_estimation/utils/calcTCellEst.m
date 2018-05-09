function TCellEst = calcTCellEst(TStart,disps)
%CALCTCELLEST Disps are quaternions.
%
% TCellEst = CALCTCELLEST(TStart,disps)
%
% TStart   - 
% disps    -
%
% TCellEst -

if ~iscell(disps)
    nDisps = size(disps,1);
    TCellDisp = cell(1,nDisps);
    for i = 1:nDisps
        T = quat2tform(disps(i,4:7));
        T(1:3,4) = disps(i,1:3);
        TCellDisp{i} = T;
    end
end

nDisps = length(TCellDisp);
TCellEst = cell(1,nDisps+1);
TCellEst{1} = TStart;
for i = 1:nDisps
    TCellEst{i+1} = TCellEst{i}*TCellDisp{i};
end
end