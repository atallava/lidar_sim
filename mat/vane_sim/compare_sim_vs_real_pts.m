% load data
relPathSimPts = 'section_08_driveby_sim_pts';
load(relPathSimPts,'simPts');

relPathPts = 'rim_stretch_veg_validation';
load(relPathPts,'pts');
realPts = pts;

%% viz
hfig = plotSimVsRealPts(realPts,simPts);
