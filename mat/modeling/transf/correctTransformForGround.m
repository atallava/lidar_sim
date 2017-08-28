function T_corrected = correctTransformForGround(T,targetObb,primitiveObb)
T_corrected = T;
delZ = targetObb.extents(3,1)-primitiveObb.extents(3,1);
T_corrected(3,4) = T_corrected(3,4) + delZ;
end