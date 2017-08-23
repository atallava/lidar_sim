relPathSimDetail = '~/lidar_sim/cpp/data/sections/section_08/hg_sim/slice_sim_detail.txt';
simDetail = loadSimDetail(relPathSimDetail);

%%
simDetailIdx = 1;

rayOrigin = simDetail.rayOrigins(simDetailIdx,:);
rayPitches = simDetail.rayPitchesCell{simDetailIdx};
rayYaws = simDetail.rayYawsCell{simDetailIdx};
realPtsAll = simDetail.realPtsAllCell{simDetailIdx};
realHitFlag = simDetail.realHitFlagCell{simDetailIdx};
simPtsAll = simDetail.simPtsAllCell{simDetailIdx};
simHitFlag = simDetail.simHitFlagCell{simDetailIdx};

%%
rayDirns = calcRayDirnsFromSph(rayPitches,rayYaws);
hfig = figure;
hold on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
box on; grid on;
scatter3(rayOrigin(1),rayOrigin(2),rayOrigin(3),'x');
rayLengthToPlot = 20;
nRays = length(rayPitches);
for i = 165:nRays
    linePts = [rayOrigin; rayOrigin+rayLengthToPlot*rayDirns(i,:)];
    plot3(linePts(:,1),linePts(:,2),linePts(:,3),'r');
    if realHitFlag(i)
        scatter3(realPtsAll(i,1),realPtsAll(i,2),realPtsAll(i,3),'bo','filled');
    else
        pt = rayOrigin+rayLengthToPlot*rayDirns(i,:);
        scatter3(pt(1),pt(2),pt(3),'bx');
    end
    if simHitFlag(i)
        scatter3(simPtsAll(i,1),simPtsAll(i,2),simPtsAll(i,3),'go','filled');
    else
        pt = rayOrigin+rayLengthToPlot*rayDirns(i,:);
        scatter3(pt(1),pt(2),pt(3),'gx');
    end
    waitforbuttonpress
end