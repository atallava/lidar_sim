% random pts + cov
nPts = 4;
pts = 10*rand(nPts,3);
meanRandPts = mean(pts,1);
covMat = cov(pts);

%% plot
[x,y,z] = genSurfXyzEllipse(covMat,meanRandPts);
surf(x,y,z,'facecolor','b','facealpha',0.1,'meshstyle','none');
hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3));
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');




