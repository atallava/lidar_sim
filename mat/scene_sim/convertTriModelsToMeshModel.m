function meshModel = convertTriModelsToMeshModel(triModels)
    meshModel.vertices = triModels.ptsFit;
    meshModel.faces = triModels.tri;
end