function dataScaled = scaleData(data,limsMat,biasIncluded)
%SCALEDATA
%
% dataScaled = SCALEDATA(data,limsMat,biasIncluded)
%
% data         - [nData,dimData(+1)] array. Possibly with bias column.
% limsMat      - [1,2] or [dimData,2] array. dimData = size(data,2)-1 if
% data has bias. [-1 1] by default.
% biasIncluded - scalar. 0 by default.
%
% dataScaled   - [nData,dimData(+1)] array. Bias column unaffected.

switch nargin
    case 1
        limsMat = [-1 1];
        biasIncluded = 0;
    case 2
        biasIncluded = 0;
    otherwise
end

dimData = size(data,2);
if biasIncluded
    dimData = dimData-1;
end

switch size(limsMat,1)
    case 1
    % use same scales for all dim
    limsMat = repmat(limsMat,dimData,1);
    case dimData
    otherwise
        msg = sprintf('%s: size(limsMat,1) is 1 or dimData',mfilename);
        error(msg);
end

dataScaled = zeros(size(data));
for i = 1:dimData
    lims = limsMat(i,:);
    vec = data(:,i);
    scale = range(lims)/range(vec);
    vecScaled = (vec-min(vec)).*scale + lims(1);
    dataScaled(:,i) = vecScaled;
end

if biasIncluded
   dataScaled(:,end) = data(:,end); 
end
end