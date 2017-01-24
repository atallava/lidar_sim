% load data
relPathSimPts = 'section_08_driveby_ground_sim_pts';
load(relPathSimPts,'simPts');

relPathPts = 'rim_stretch_ground_validation';
load(relPathPts,'pts');
realPts = pts;

%% viz
hfig = plotComparePts(realPts,simPts);
