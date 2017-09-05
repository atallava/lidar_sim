function dataWhitened = whitenData(data,biasIncluded)
%WHITENDATA
%
% dataWhitened = WHITENDATA(data,biasIncluded)
%
% data         - [nData,dimData(+1)] array. Possibly with bias column.
% biasIncluded - scalar. 0 by default.
%
% dataWhitened - [nData,dimData(+1)] array. Bias column unaffected.

if nargin < 2
    biasIncluded = 0;
end

if biasIncluded
    % assuming bias is last column
    biasCol = data(:,end);
    data = data(:,(1:end-1));
end

covMat = cov(data);
covMatInv = inv(covMat);
W = chol(covMatInv);
dataWhitened = data*W';

if biasIncluded
    dataWhitened = [dataWhitened biasCol];
end
end