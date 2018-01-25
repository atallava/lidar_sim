function betaAvg = calcAvgViewAngle(sMax,sQv,sPerp,vQv)
%CALCAVGVIEWANGLE
%
% betaAvg = CALCAVGVIEWANGLE(sMax,sQv,sPerp,vQv)
%
% sMax    -
% sQv     -
% sPerp   -
% vQv     -
%
% betaAvg -
dt = 0.1;

tQv = 2*sMax/vQv;
tVec = 0:dt:tQv;
sVec = -sMax + tVec*vQv;
betaVec = zeros(size(sVec));
for i = 1:length(sVec)
   betaVec(i) = calcViewAngle(sVec(i),sQv,sPerp); 
end
betaAvg = mean(betaVec);
end