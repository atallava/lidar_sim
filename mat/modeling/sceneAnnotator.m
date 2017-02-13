function annotations = sceneAnnotator(pts)

%% set up figure

% set figure to full screen
figure('units','normalized','outerposition',[0 0 1 1])

scatter3(pts(:,1),pts(:,2),pts(:,3),'.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');

% this helps axis take up screen when zooming in
set(gcf,'PaperPositionMode', 'auto');
set(gcf, 'ResizeFcn', 'resizeFcn');


set(gcf,'KeyPressFcn',@keyPressFcn);

az = 1;
el = 90;
view([az el]);

xlabel('x'); ylabel('y'); zlabel('z');

%% define control keys
keys = struct(...
    'record','r', ...
    'kill','q');

fprintf('%s : record\n',keys.record);
fprintf('%s : kill\n',keys.kill);
fprintf('class legend:\n 1: low shrub\n 2: low shrub patch\n 3: medium shrub\n 4: medium shrub patch\n 5: thin shrub\n 6: large shrub\n 7: large shrub patch\n 8: medium tree\n 9: large tree\n');

%% initialize
state = 1;
annotations = struct('x',{},'y',{},'class',{});
tPause = 5e-2;

while true
    if state
         pause(tPause);
         continue;
    else
        break;
    end
    pause(tPause);
end

%% keypress callback
    function keyPressFcn(hObj,event)
        switch event.Key
            case keys.kill
                state = 0;
            case keys.record
                [x,y] = ginput;
                class = input('class name: ');
                for i = 1:length(x)
                    hold on;
                    plot3(x(i),y(i),0,'rx','markersize',20,'linewidth',5);
                    ann.x = x(i);
                    ann.y = y(i);
                    ann.class = class;
                    annotations(end+1) = ann;
                end
            otherwise
                % do nothing
        end
    end

end
     