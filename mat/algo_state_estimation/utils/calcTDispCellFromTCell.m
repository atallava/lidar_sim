function TDispCell = calcTDispCellFromTCell(TCell)
%CALCTDISPCELLFROMTCELL Given cell of transforms, calc the relative
% displacement transforms.
%
% TDispCell = CALCTDISPCELLFROMTCELL(TCell)
%
% TCell     -
%
% TDispCell -

nTs = length(TCell);
nTDisps = nTs - 1;
TDispCell = cell(1, nTDisps);

for i = 1:nTDisps
    TDispCell{i} = TCell{i}\TCell{i+1};
end
end