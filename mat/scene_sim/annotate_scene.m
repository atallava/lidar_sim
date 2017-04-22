relPathPts = '../data/section_pts_04_world_frame_subsampled';
load(relPathPts,'pts');

%% 
annotations = sceneAnnotator(pts);

%%
relPathAnnotations = '../data/section_04_annotations.txt';
writeSceneAnnotations(annotations,relPathAnnotations);
