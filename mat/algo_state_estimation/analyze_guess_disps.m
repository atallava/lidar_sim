% viz the traj after guess disps

sectionId = 4;
scansVersion = '300118';
dataSource = 'real';
sourceVersion = '';
guessDispsVersion = '18220218';

%% load scan poses
relPathScanPoses = genRelPathScanPoses(sectionId, scansVersion, dataSource, sourceVersion);
[tPoses,TCellReal] = loadScanPoses(relPathScanPoses);

nPoses = length(TCellReal);
posnsReal = zeros(nPoses,3);
for i = 1:nPoses
    TReal = TCellReal{i};
    posnsReal(i,:) = TReal(1:3,4);
end

%% load guess disps
relPathGuessDisps = genRelPathGuessDisps(sectionId, scansVersion, guessDispsVersion);
[~,guessDisps] = loadGuessDisps(relPathGuessDisps);
TCellGuess = calcTCellEst(TCellReal{1},guessDisps);
posnsGuess = getPosnsFromTfs(TCellGuess);

%% viz
hfig = figure();
lineWidth = 1.5;
plot3(posnsReal(:,1),posnsReal(:,2),posnsReal(:,3),'b-', ...
    'linewidth',lineWidth);
hold on; axis equal;
plot3(posnsGuess(:,1),posnsGuess(:,2),posnsGuess(:,3),'r-', ...
    'linewidth',lineWidth);
xlabel('x (m)'); ylabel('y (m)');
legend({'real','est'});
view(2);
box on; grid on;




