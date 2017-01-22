% random pts + cov
nRandPts = 20;
randPts = 10*rand(nRandPts,3);
meanRandPts = mean(randPts,1);
randPts = bsxfun(@minus,randPts,meanRandPts);
covMat = cov(randPts);

%% plot
[x,y,z] = genXyzEllipse(covMat);
surf(x,y,z,'facecolor','b','facealpha',0.1,'meshstyle','none');
hold on;
scatter3(randPts(:,1),randPts(:,2),randPts(:,3));
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');




