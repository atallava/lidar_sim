function dataCentered = centerData(data,biasIncluded)
%CENTERDATA
%
% dataCentered = CENTERDATA(data,biasIncluded)
%
% data         - [nData,dimData(+1)] array. Possibly with bias column.
% biasIncluded - scalar. 0 by default.
%
% dataCentered - [nData,dimData(+1)] array. Bias column unaffected.

if nargin < 2
    biasIncluded = 0;
end

if biasIncluded
    % assuming bias is at the end
    biasCol = data(:,end);
    data = data(:,1:end-1);
end
dataMean = mean(data,1);
dataCentered = bsxfun(@minus,data,dataMean);

if biasIncluded
    dataCentered = [dataCentered biasCol];
end
end