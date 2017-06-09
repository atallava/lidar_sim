hfig = figure;
hold on;
axis equal;
lineWidth = 10;

% x axis
plot3([0 1],[0 0],[0 0],'r','linewidth',lineWidth);

% y axis
plot3([0 0],[0 1],[0 0],'g','linewidth',lineWidth);

% z axis
plot3([0 0],[0 0],[0 1],'b','linewidth',lineWidth);

set(gca,'visible','off');


