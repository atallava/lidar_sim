% random pts
nPts = 10;
pts = rand(nPts,2)*10;
tape = calcPtsTape(pts);

%% viz
figure;
axis equal;
box on; grid on;
scatter(pts(:,1),pts(:,2),'b.');
hold on;
for i = 2:length(tape)
    id1 = tape(i-1);
    id2 = tape(i);
    plot(pts([id1 id2],1),pts([id1 id2],2),'r');
end
id1 = tape(end);
id2 = tape(1);
plot(pts([id1 id2],1),pts([id1 id2],2),'r');
