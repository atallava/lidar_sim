function [balancedData1,balancedData2] = balanceTwoArrays(data1,data2,nMaxData)
    nData1 = size(data1,1);
    nData2 = size(data2,1);
    
    if nargin < 3
        nMaxData = min(nData1,nData2);
    end
   
    if nData1 < nMaxData
        balancedIds1 = 1:nData1;
    else
        balancedIds1 = floor(linspace(1,nData1,nMaxData));
    end
    if nData2 < nMaxData
        balancedIds2 = 1:nData2;
    else
        balancedIds2 = floor(linspace(1,nData2,nMaxData));
    end
    
    balancedData1 = data1(balancedIds1,:);
    balancedData2 = data2(balancedIds2,:);
end