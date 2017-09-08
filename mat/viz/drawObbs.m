function drawObbs(hfig,obbCell,ptsCell)
%DRAWOBBS
%
% DRAWOBBS(hfig,obbCell,ptsCell)
%
% hfig    -
% obbCell -
% ptsCell -

if nargin < 3
    ptsInput = false;
else
    ptsInput = true;
end

nObbs = length(obbCell);
for i = 1:nObbs
    if ptsInput
        drawObb(hfig,obbCell{i},ptsCell{i});
    else
        drawObb(hfig,obbCell{i});
    end
end

end