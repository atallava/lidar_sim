relPathXY = 'fixed_xy';
load(relPathXY,'xyLog','tLog');

%%
hfig = figure;
plot(xyLog(:,1),xyLog(:,2));
hold on;
axis equal;

hCursor = plot(nan,nan,'ro');

%%
nLog = length(tLog);
tMax = tLog(end);
t = rand()*tMax;
dataId = indexOfNearestTime(t,tLog);
xy = xyLog(dataId,:);
set(hCursor,'XData',xy(1),'YData',xy(2));
title(sprintf('time: %.2f',t));


%%
for i = 1:100:nLog
    xy = xyLog(i,:);
    set(hCursor,'XData',xy(1),'YData',xy(2));
    title(sprintf('%d',i));
    pause(1e-3);
end
