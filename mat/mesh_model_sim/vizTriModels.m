function hfig = vizTriModels(triModels)
    meshModel = convertTriModelsToMeshModel(triModels);
    hfig = vizMesh(meshModel);
end