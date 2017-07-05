figure; hold on; axis equal;
% plot pts
scatter3(pts(:,1),pts(:,2),pts(:,3),'.r');

% plot unrolled pts
unrolledHitFlag = logical(unrolledHitFlag);
unrolledHitPts = unrolledPts(unrolledHitFlag,:);
scatter3(unrolledHitPts(:,1),unrolledHitPts(:,2),unrolledHitPts(:,3),'xb');

% plot the yaw and pitch lines
for i = 1:length(unrolledHitFlag)
    if ~unrolledHitFlag(i)
        continue;
    else
        [u,v,w] = sph2cart(unrolledYaw(i),unrolledPitch(i),5);
        plot3([0 u],[0 v],[0 w],'g');
    end
end