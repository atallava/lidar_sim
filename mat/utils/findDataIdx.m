function [idx,matchVec] = findDataIdx(mat,query)
%FINDDATAIDX Returns closest match in matrix to query vector.
%
% [idx,matchVec] = FINDDATAIDX(mat,query)
%
% mat      - [nData,dimData] array.
% query    - nData length vector.
%
% idx      - scalar.
% matchVec - [1,dimData] array.

query = flipVecToRow(query);
dr = bsxfun(@minus,mat,query);
ds2 = sum(dr.^2,2);
ds = sqrt(ds2);
[minDist,idx] = min(ds);
matchVec = mat(idx,:);
end